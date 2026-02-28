#!/usr/bin/env python3
"""
Fronius GEN24 Battery Control via Modbus TCP

Controls battery charge/discharge modes by writing SunSpec Storage registers
directly over Modbus TCP. No external dependencies - uses only Python stdlib.

Tested on:
    Fronius Symo GEN24 10.0, software revision ROW 1.39.5-1

Usage:
    python3 fronius_battery_control.py --host 192.168.1.100 --action force_charge [--rate 3000] [--rvrt 900]
    python3 fronius_battery_control.py --host 192.168.1.100 --action force_discharge [--rate 3000] [--rvrt 900]
    python3 fronius_battery_control.py --host 192.168.1.100 --action hold [--rvrt 900]
    python3 fronius_battery_control.py --host 192.168.1.100 --action reset
    python3 fronius_battery_control.py --host 192.168.1.100 --action status

Actions:
    force_charge    - Force battery to charge from grid at specified rate
    force_discharge - Force battery to discharge at specified rate
    hold            - Prevent battery from discharging (solar charging still allowed)
    reset           - Return to automatic/default mode
    status          - Read and display current register values

Options:
    --host HOST     Inverter IP address (required)
    --port PORT     Modbus TCP port (default: 502)
    --slave SLAVE   Modbus slave ID (default: 1)
    --rate WATTS    Charge/discharge rate in watts (default: inverter max)
    --rvrt SECONDS  Revert timer in seconds (default: 900).
                    The inverter will automatically return to auto mode if no
                    further writes are received within this window. This is a
                    safety feature - if the calling process stops, the inverter
                    recovers on its own. Set to 0 to disable (not recommended).
    --no-verify     Skip read-back verification after writing
    --json          Output status as JSON
    --retries N     Number of retries on connection failure (default: 3)

Register addressing note:
    This script sends SunSpec register addresses directly on the wire without
    subtracting an offset. This is correct for the Fronius GEN24 but may differ
    from other Modbus devices where a -1 or -40001 offset is expected. If you
    are adapting this for another inverter, verify the addressing convention
    against your inverter's Modbus documentation before writing any registers.
"""

import argparse
import json
import socket
import struct
import sys
import time

# =============================================================================
# SunSpec Storage Register Map - Fronius Symo GEN24 10.0 (ROW 1.39.5-1)
#
# Addresses are absolute Modbus register addresses sent directly on the wire.
# Verified against live system 2026-02-14.
# =============================================================================
REG_WCHAMAX      = 40345  # Max charge rate (W), read-only reference
REG_STORCTL_MOD  = 40348  # Storage control mode bitmask
REG_MIN_RSV_PCT  = 40350  # Minimum reserve SoC (%)
REG_CHASTATE     = 40351  # Current SoC (scaled, SF at 40365, typically -2)
REG_CHAST        = 40354  # Charge status enum (see CHAST_NAMES below)
REG_OUTWRTE      = 40355  # Discharge rate limit (% of WChaMax, signed, SF at 40368)
REG_INWRTE       = 40356  # Charge rate limit (% of WChaMax, signed, SF at 40368)
REG_RVRT_TMS     = 40358  # Revert timer (seconds, 0 = disabled)
REG_CHAGRISET    = 40360  # Grid charging enable (1 = enabled)
REG_INOUTWRTE_SF = 40368  # Scale factor for InWRte/OutWRte (typically -2)

# StorCtl_Mod bitmask values
STORCTL_CHARGE_LIMIT    = 1  # Bit 0: apply InWRte limit
STORCTL_DISCHARGE_LIMIT = 2  # Bit 1: apply OutWRte limit

# Fallback WChaMax if the register reads as 0 or 65535
DEFAULT_WCHAMAX = 25600

# Charge status enum descriptions
CHAST_NAMES = {
    1: "Off",
    2: "Empty",
    3: "Discharging",
    4: "Charging",
    5: "Full",
    6: "Holding",
    7: "Testing",
}


# =============================================================================
# Raw Modbus TCP Implementation
# =============================================================================
class ModbusTCPError(Exception):
    """Raised on any Modbus communication or protocol error."""
    pass


class ModbusTCPClient:
    """
    Minimal Modbus TCP client using raw sockets. No external dependencies.

    Implements function codes 0x03 (read holding registers) and 0x06/0x10
    (write single/multiple holding registers).

    Note on addressing: register addresses are sent to the wire as-is (no
    offset subtraction). This matches Fronius GEN24 behaviour but may need
    adjustment for other devices.
    """

    def __init__(self, host, port=502, slave_id=1, timeout=5):
        self.host = host
        self.port = port
        self.slave_id = slave_id
        self.timeout = timeout
        self._sock = None
        self._transaction_id = 0

    def connect(self):
        """Open TCP connection to inverter."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)
        self._sock.connect((self.host, self.port))

    def close(self):
        """Close TCP connection."""
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def _next_transaction_id(self):
        self._transaction_id = (self._transaction_id + 1) & 0xFFFF
        return self._transaction_id

    def _build_mbap_header(self, length):
        """Build Modbus Application Protocol (MBAP) header."""
        tid = self._next_transaction_id()
        # Transaction ID (2) + Protocol ID (2, always 0) + Length (2) + Unit ID (1)
        return struct.pack('>HHHB', tid, 0, length, self.slave_id), tid

    def _send_and_receive(self, pdu):
        """Send a Modbus PDU and return the response PDU."""
        header, tid = self._build_mbap_header(len(pdu) + 1)
        self._sock.sendall(header + pdu)

        resp_header = self._recv_exact(7)
        resp_tid, resp_proto, resp_len, resp_unit = struct.unpack('>HHHB', resp_header)

        if resp_tid != tid:
            raise ModbusTCPError(f"Transaction ID mismatch: sent {tid}, got {resp_tid}")

        resp_pdu = self._recv_exact(resp_len - 1)

        if resp_pdu[0] & 0x80:
            exception_code = resp_pdu[1]
            exceptions = {
                1: "Illegal Function",
                2: "Illegal Data Address",
                3: "Illegal Data Value",
                4: "Server Device Failure",
            }
            msg = exceptions.get(exception_code, f"Unknown ({exception_code})")
            raise ModbusTCPError(f"Modbus exception: {msg}")

        return resp_pdu

    def _recv_exact(self, n):
        """Receive exactly n bytes, blocking until complete."""
        data = b''
        while len(data) < n:
            chunk = self._sock.recv(n - len(data))
            if not chunk:
                raise ModbusTCPError("Connection closed by remote")
            data += chunk
        return data

    def read_holding_registers(self, address, count):
        """Read holding registers (function code 0x03).

        Args:
            address: Starting register address (sent directly on wire)
            count:   Number of registers to read

        Returns:
            List of unsigned 16-bit integer values
        """
        pdu = struct.pack('>BHH', 0x03, address, count)
        resp = self._send_and_receive(pdu)

        byte_count = resp[1]
        if byte_count != count * 2:
            raise ModbusTCPError(f"Expected {count*2} data bytes, got {byte_count}")

        return [
            struct.unpack('>H', resp[2 + i*2 : 4 + i*2])[0]
            for i in range(count)
        ]

    def write_register(self, address, value):
        """Write a single holding register (function code 0x06).

        Args:
            address: Register address (sent directly on wire)
            value:   Value to write (unsigned 16-bit, 0-65535)
        """
        value = value & 0xFFFF
        pdu = struct.pack('>BHH', 0x06, address, value)
        resp = self._send_and_receive(pdu)

        resp_addr, resp_val = struct.unpack('>HH', resp[1:5])
        if resp_addr != address:
            raise ModbusTCPError(f"Address mismatch in write echo: sent {address}, got {resp_addr}")

    def write_registers(self, address, values):
        """Write multiple holding registers (function code 0x10).

        Args:
            address: Starting register address (sent directly on wire)
            values:  List of unsigned 16-bit integers
        """
        count = len(values)
        byte_count = count * 2
        pdu = struct.pack('>BHHB', 0x10, address, count, byte_count)
        for v in values:
            pdu += struct.pack('>H', v & 0xFFFF)

        resp = self._send_and_receive(pdu)
        resp_addr, resp_count = struct.unpack('>HH', resp[1:5])
        if resp_count != count:
            raise ModbusTCPError(f"Write count mismatch: sent {count}, got {resp_count}")


# =============================================================================
# Register Value Conversions
# =============================================================================
def watts_to_register_pct(watts, wchamax, scale_factor=-2):
    """Convert watts to a signed register percentage value.

    InWRte and OutWRte store percentage of WChaMax, scaled by InOutWRte_SF.
    With SF=-2, the register unit is 0.01%, so 10000 = 100.00% = full rate.

    Args:
        watts:        Power in watts
        wchamax:      Inverter maximum charge power in watts
        scale_factor: InOutWRte_SF register value (typically -2)

    Returns:
        Unsigned integer suitable for writing to InWRte/OutWRte
    """
    if wchamax <= 0:
        return 10000
    pct = max(0.0, min(100.0, (watts / wchamax) * 100.0))
    return int(pct * (10 ** (-scale_factor)))


def register_pct_to_watts(reg_value, wchamax, scale_factor=-2):
    """Convert a signed register percentage value back to watts."""
    pct = reg_value / (10 ** (-scale_factor))
    return pct * wchamax / 100.0


def signed_to_unsigned_16(value):
    """Convert a signed integer to unsigned 16-bit (two's complement)."""
    if value < 0:
        return value + 65536
    return value & 0xFFFF


def unsigned_to_signed_16(value):
    """Convert an unsigned 16-bit Modbus value to signed."""
    return value - 65536 if value >= 32768 else value


# =============================================================================
# Battery Control
# =============================================================================
def read_status(client):
    """Read and return current battery control state from inverter."""
    # Block read: 24 registers from REG_WCHAMAX covers everything we need
    values = client.read_holding_registers(REG_WCHAMAX, 24)

    wchamax      = values[REG_WCHAMAX      - REG_WCHAMAX]
    storctl_mod  = values[REG_STORCTL_MOD  - REG_WCHAMAX]
    min_rsv_pct  = values[REG_MIN_RSV_PCT  - REG_WCHAMAX]
    chastate     = values[REG_CHASTATE     - REG_WCHAMAX]
    chast        = values[REG_CHAST        - REG_WCHAMAX]
    outwrte_raw  = values[REG_OUTWRTE      - REG_WCHAMAX]
    inwrte_raw   = values[REG_INWRTE       - REG_WCHAMAX]
    rvrt_tms     = values[REG_RVRT_TMS     - REG_WCHAMAX]
    chagriset    = values[REG_CHAGRISET    - REG_WCHAMAX]
    sf           = unsigned_to_signed_16(values[REG_INOUTWRTE_SF - REG_WCHAMAX])

    outwrte = unsigned_to_signed_16(outwrte_raw)
    inwrte  = unsigned_to_signed_16(inwrte_raw)

    sf_multiplier = 10 ** (-sf) if sf != 0 else 100
    outwrte_pct   = outwrte / sf_multiplier
    inwrte_pct    = inwrte  / sf_multiplier
    outwrte_watts = outwrte_pct * wchamax / 100.0
    inwrte_watts  = inwrte_pct  * wchamax / 100.0

    # SoC SF is at 40365 (index 20 in the block), hardcoded as -2 here.
    # The raw value is divided by 100 to give percentage.
    soc = chastate / 100.0

    # Determine operating mode from register state
    if storctl_mod == 0:
        mode = "auto"
    elif storctl_mod == STORCTL_DISCHARGE_LIMIT and outwrte < 0:
        mode = "force_charge"
    elif storctl_mod == 3 and inwrte < 0:
        mode = "force_discharge"
    elif storctl_mod == STORCTL_DISCHARGE_LIMIT and outwrte == 0:
        mode = "hold"
    else:
        mode = f"custom (StorCtl_Mod={storctl_mod})"

    return {
        "mode":              mode,
        "storctl_mod":       storctl_mod,
        "wchamax_w":         wchamax,
        "soc_pct":           round(soc, 1),
        "outwrte_raw":       outwrte,
        "outwrte_pct":       round(outwrte_pct, 2),
        "outwrte_watts":     round(outwrte_watts, 1),
        "inwrte_raw":        inwrte,
        "inwrte_pct":        round(inwrte_pct, 2),
        "inwrte_watts":      round(inwrte_watts, 1),
        "rvrt_tms":          rvrt_tms,
        "min_rsv_pct":       min_rsv_pct / 100.0,
        "charge_status":     CHAST_NAMES.get(chast, f"unknown({chast})"),
        "charge_status_raw": chast,
        "grid_charging":     chagriset == 1,
        "scale_factor":      sf,
    }


def apply_control(client, action, rate_watts=None, rvrt_seconds=900):
    """Write control registers to set the battery operating mode.

    Args:
        client:       ModbusTCPClient instance (must be connected)
        action:       'force_charge', 'force_discharge', 'hold', or 'reset'
        rate_watts:   Power in watts (None = inverter maximum)
        rvrt_seconds: Revert timer in seconds (0 = no auto-revert)
    """
    values = client.read_holding_registers(REG_WCHAMAX, 1)
    wchamax = values[0]
    if wchamax == 0 or wchamax == 65535:
        wchamax = DEFAULT_WCHAMAX

    if rate_watts is None:
        rate_watts = wchamax
    rate_watts = min(rate_watts, wchamax)
    rate_pct_value = watts_to_register_pct(rate_watts, wchamax)

    # Brief pause between read and write (Fronius docs recommend sequential requests)
    time.sleep(0.5)

    if action == "force_charge":
        # Force charge from grid:
        #   ChaGriSet = 1       enable grid as charge source
        #   StorCtl_Mod = 2     apply discharge limit (bit 1)
        #   OutWRte = -rate     negative value creates a charge floor
        #   InWRte = 10000      allow full charge rate
        outwrte_value = signed_to_unsigned_16(-rate_pct_value)
        writes = [
            (REG_CHAGRISET,   1),
            (REG_RVRT_TMS,    rvrt_seconds),
            (REG_OUTWRTE,     outwrte_value),
            (REG_INWRTE,      10000),
            (REG_STORCTL_MOD, STORCTL_DISCHARGE_LIMIT),
        ]

    elif action == "force_discharge":
        # Force discharge to grid:
        #   StorCtl_Mod = 3     apply both charge and discharge limits (bits 0+1)
        #   InWRte = -rate      negative InWRte creates a discharge floor
        #   OutWRte = +rate     positive OutWRte caps discharge ceiling
        #   Together these create a power window forcing active discharge.
        inwrte_value = signed_to_unsigned_16(-rate_pct_value)
        writes = [
            (REG_RVRT_TMS,    rvrt_seconds),
            (REG_INWRTE,      inwrte_value),
            (REG_OUTWRTE,     rate_pct_value),
            (REG_STORCTL_MOD, 3),
        ]

    elif action == "hold":
        # Hold (prevent discharge, allow solar charging):
        #   StorCtl_Mod = 2     apply discharge limit (bit 1)
        #   OutWRte = 0         0% discharge allowed
        #   InWRte = 10000      100% charge from solar allowed
        writes = [
            (REG_RVRT_TMS,    rvrt_seconds),
            (REG_OUTWRTE,     0),
            (REG_INWRTE,      10000),
            (REG_STORCTL_MOD, STORCTL_DISCHARGE_LIMIT),
        ]

    elif action == "reset":
        # Return to automatic mode:
        #   StorCtl_Mod = 0     no limits active
        #   OutWRte = 10000     100% discharge allowed
        #   InWRte = 10000      100% charge allowed
        #   RvrtTms = 0         no revert timer needed
        writes = [
            (REG_STORCTL_MOD, 0),
            (REG_OUTWRTE,     10000),
            (REG_INWRTE,      10000),
            (REG_RVRT_TMS,    0),
        ]

    else:
        raise ValueError(f"Unknown action: {action}")

    for reg, value in writes:
        client.write_register(reg, value)
        time.sleep(0.3)  # Fronius requires a small gap between sequential writes

    return {
        "action":             action,
        "rate_watts":         rate_watts,
        "rate_pct":           round(rate_watts / wchamax * 100, 1),
        "wchamax":            wchamax,
        "rvrt_seconds":       rvrt_seconds,
        "registers_written":  writes,
    }


def verify_write(client, action, max_attempts=3, delay=2):
    """Read back registers and confirm the action took effect.

    Returns:
        Tuple of (success: bool, status: dict)
    """
    for _ in range(max_attempts):
        time.sleep(delay)
        status = read_status(client)

        if action == "reset"          and status["storctl_mod"] == 0:
            return True, status
        if action == "force_charge"   and status["storctl_mod"] == STORCTL_DISCHARGE_LIMIT:
            return True, status
        if action == "force_discharge" and status["storctl_mod"] == 3:
            return True, status
        if action == "hold"           and status["storctl_mod"] == STORCTL_DISCHARGE_LIMIT:
            return True, status

    return False, status


# =============================================================================
# Entry Point
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Fronius GEN24 Battery Control via Modbus TCP",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--action",   required=True,
                        choices=["force_charge", "force_discharge", "hold", "reset", "status"],
                        help="Control action to perform")
    parser.add_argument("--host",     required=True,
                        help="Inverter IP address")
    parser.add_argument("--port",     type=int, default=502,
                        help="Modbus TCP port (default: 502)")
    parser.add_argument("--slave",    type=int, default=1,
                        help="Modbus slave ID (default: 1)")
    parser.add_argument("--rate",     type=int, default=None,
                        help="Charge/discharge rate in watts (default: inverter max)")
    parser.add_argument("--rvrt",     type=int, default=900,
                        help="Revert timer in seconds (default: 900, 0=disabled)")
    parser.add_argument("--no-verify", action="store_false", dest="verify",
                        help="Skip write verification")
    parser.add_argument("--json",     action="store_true",
                        help="Output as JSON")
    parser.add_argument("--retries",  type=int, default=3,
                        help="Connection retries (default: 3)")

    parser.set_defaults(verify=True)
    args = parser.parse_args()

    client = ModbusTCPClient(args.host, args.port, args.slave)

    last_error = None
    for attempt in range(args.retries):
        try:
            client.connect()

            if args.action == "status":
                status = read_status(client)
                if args.json:
                    print(json.dumps(status, indent=2))
                else:
                    print(f"Mode:           {status['mode']}")
                    print(f"SoC:            {status['soc_pct']}%")
                    print(f"Charge Status:  {status['charge_status']}")
                    print(f"StorCtl_Mod:    {status['storctl_mod']}")
                    print(f"OutWRte:        {status['outwrte_pct']}% ({status['outwrte_watts']}W)")
                    print(f"InWRte:         {status['inwrte_pct']}% ({status['inwrte_watts']}W)")
                    print(f"WChaMax:        {status['wchamax_w']}W")
                    print(f"Revert Timer:   {status['rvrt_tms']}s")
                    print(f"Min Reserve:    {status['min_rsv_pct']}%")
                    print(f"Grid Charging:  {status['grid_charging']}")
            else:
                result = apply_control(client, args.action, args.rate, args.rvrt)

                if args.verify:
                    verified, status = verify_write(client, args.action)
                    result["verified"] = verified
                    result["current_status"] = status

                    if not verified:
                        print(f"WARNING: Write verification failed! StorCtl_Mod={status['storctl_mod']}",
                              file=sys.stderr)
                        client.close()
                        sys.exit(2)

                if args.json:
                    print(json.dumps(result, indent=2))
                else:
                    print(f"Action:         {result['action']}")
                    print(f"Rate:           {result['rate_watts']}W ({result['rate_pct']}% of {result['wchamax']}W)")
                    print(f"Revert Timer:   {result['rvrt_seconds']}s")
                    if args.verify:
                        print(f"Verified:       {result['verified']}")
                        cs = result['current_status']
                        print(f"Current Mode:   {cs['mode']}")
                        print(f"Current SoC:    {cs['soc_pct']}%")

            client.close()
            sys.exit(0)

        except (socket.error, ModbusTCPError) as e:
            last_error = e
            client.close()
            if attempt < args.retries - 1:
                wait = (attempt + 1) * 2
                print(f"Attempt {attempt+1} failed: {e}. Retrying in {wait}s...", file=sys.stderr)
                time.sleep(wait)

        except Exception as e:
            client.close()
            print(f"Unexpected error: {e}", file=sys.stderr)
            sys.exit(1)

    print(f"All {args.retries} attempts failed. Last error: {last_error}", file=sys.stderr)
    sys.exit(1)


if __name__ == "__main__":
    main()
