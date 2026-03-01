# fronius-modbus-control

A Python script for controlling a Fronius GEN24 hybrid inverter's battery charge and discharge behaviour via Modbus TCP. No external dependencies — uses only the Python standard library.

## Background

Fronius GEN24 inverters expose a SunSpec-compliant Modbus TCP interface for battery control, but getting it to work reliably has a few non-obvious gotchas. This script documents what actually works, including the register addresses, the signing conventions for rate registers, and the revert timer behaviour that will silently undo your writes if you don't account for it.

**Tested on:** Fronius Symo GEN24 10.0, software revision ROW 1.39.5-1

It is used as the Modbus control layer in a [Predbat](https://github.com/springfall2008/batpred) integration for Fronius inverters. See [Using with Predbat](#using-with-predbat) below if that's why you're here.

---

## ⚠️ Safety Warning

This script writes directly to inverter control registers and can cause the battery to charge from the grid or discharge to the grid. Before using it in any automation:

- **Run on an isolated LAN.** Modbus TCP has no authentication. Anyone on the same network can send register writes to your inverter.
- **Understand the financial impact.** Unintended force charge or force discharge during expensive rate periods will cost you money.
- **Test manually first.** Use `--action status` to verify communication, then test each action individually before putting it into automation.
- **Use the revert timer.** The default `--rvrt 900` means the inverter returns to auto mode after 15 minutes if no further writes arrive. Don't disable this unless you have a good reason.
- **The register behaviour documented here is specific to the tested firmware.** Fronius may change register behaviour across firmware versions. If something isn't working as expected, check your firmware version and verify the register map.

---

## Prerequisites

Before using this script, you need to enable and configure Modbus TCP on your Fronius inverter. In the Fronius web interface, go to **Communication → Modbus** and set the following:

| Setting | Value |
|---------|-------|
| Mode | TCP Server |
| Port | 502 (default) |
| Sunspec Model Type | **int + SF** |
| Meter Address Offset | 200 |
| Allow Control via Modbus | On |
| Restrict Control | Off |

> **The `Sunspec Model Type` setting is critical.** This must be set to `int + SF` (integer with separate scale factor registers). The entire register map used by this script — including all register addresses, value formats, and scale factor registers like `InOutWRte_SF` (40368) — assumes `int + SF` mode. If your inverter is set to `float` mode instead, the register addresses will be different and the script will not work correctly.

---

## The RvrtTms Gotcha

If you've tried to control a Fronius GEN24 via Modbus before and found your writes silently reverting after a second or two, this is almost certainly why.

The inverter has a register called `InOutWRte_RvrtTms` (40358) — a revert timer. When set to a non-zero value, the inverter will automatically return all control registers to their defaults after that many seconds if no new write arrives. This is intended as a safety feature, but the default value may not be what you expect, and if it's set very low (e.g. 2 seconds), writes appear to work momentarily and then silently undo themselves.

**Fix:** If you're seeing this behaviour, go to the Fronius web interface → Communication → Modbus and click **Reset Modbus API Controls**. This resets `RvrtTms` to its factory default. The script then manages the revert timer explicitly on every write.

---

## How It Works

The script writes to a small set of SunSpec Storage Model registers to place the battery into one of four operating modes:

| Mode | Description |
|------|-------------|
| `force_charge` | Charge battery from grid at specified rate |
| `force_discharge` | Discharge battery to grid at specified rate |
| `hold` | Prevent discharge; battery can still charge from solar |
| `reset` | Return to automatic (self-consumption) mode |

Control is via three key registers:

- **`StorCtl_Mod`** (40348) — bitmask that activates charge and/or discharge limiting
- **`InWRte`** (40356) — charge rate limit as a signed percentage of `WChaMax`
- **`OutWRte`** (40355) — discharge rate limit as a signed percentage of `WChaMax`

The sign conventions are the non-obvious part. A **negative** `OutWRte` creates a charge floor (used in `force_charge`), and a **negative** `InWRte` creates a discharge floor (used in `force_discharge`). The SunSpec spec documents this, but it's easy to miss.

---

## Register Addressing

This script sends SunSpec register addresses directly on the wire without subtracting any offset. This is correct for the Fronius GEN24 — register 40345 is sent as address 40345 in the Modbus request.

This differs from some Modbus devices and libraries where a -1 or -40001 offset is applied before sending. If you are adapting this for a different inverter, verify the addressing convention against your inverter's Modbus documentation before writing any control registers.

Note that the register addresses used throughout this documentation assume the `Sunspec Model Type` is set to `int + SF` (see [Prerequisites](#prerequisites)). The `float` model type uses a different register layout.

---

## Usage

```
python3 fronius_battery_control.py --host <IP> --action <action> [options]
```

`--host` is required. There is no default — you must specify your inverter's IP address explicitly.

### Actions

```bash
# Check current state
python3 fronius_battery_control.py --host 192.168.1.100 --action status

# Force charge from grid at 3000W for up to 15 minutes
python3 fronius_battery_control.py --host 192.168.1.100 --action force_charge --rate 3000

# Force charge at maximum inverter rate
python3 fronius_battery_control.py --host 192.168.1.100 --action force_charge

# Force discharge at 2000W
python3 fronius_battery_control.py --host 192.168.1.100 --action force_discharge --rate 2000

# Hold (prevent discharge, solar charging still works)
python3 fronius_battery_control.py --host 192.168.1.100 --action hold

# Return to automatic mode
python3 fronius_battery_control.py --host 192.168.1.100 --action reset

# JSON output (useful for scripting)
python3 fronius_battery_control.py --host 192.168.1.100 --action status --json
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--host` | required | Inverter IP address |
| `--port` | 502 | Modbus TCP port |
| `--slave` | 1 | Modbus slave ID |
| `--rate` | inverter max | Charge/discharge rate in watts |
| `--rvrt` | 900 | Revert timer in seconds (0 to disable) |
| `--no-verify` | — | Skip read-back verification after writing |
| `--json` | — | Output as JSON |
| `--retries` | 3 | Connection retry attempts |

---

## Register Map

| Register | Name | Description |
|----------|------|-------------|
| 40345 | `WChaMax` | Max charge rate (W), read-only |
| 40348 | `StorCtl_Mod` | Control mode bitmask (bit 0 = charge limit, bit 1 = discharge limit) |
| 40350 | `MinRsvPct` | Minimum reserve SoC (%) |
| 40351 | `ChaState` | Current SoC (SF at 40365, typically -2) |
| 40354 | `ChaSt` | Charge status enum |
| 40355 | `OutWRte` | Discharge rate limit (% of WChaMax, signed, SF at 40368) |
| 40356 | `InWRte` | Charge rate limit (% of WChaMax, signed, SF at 40368) |
| 40358 | `InOutWRte_RvrtTms` | Revert timer (seconds) |
| 40360 | `ChaGriSet` | Grid charging enable (1 = enabled) |
| 40368 | `InOutWRte_SF` | Scale factor for InWRte/OutWRte (typically -2) |

---

## Using with Predbat

[Predbat](https://github.com/springfall2008/batpred) is a Home Assistant battery prediction and optimisation app. It doesn't natively support Fronius inverters, but can control them via its `has_service_api` mechanism — Predbat writes to Home Assistant `input_boolean` and `input_number` helpers, HA automations watch those helpers, and they call this script via a `shell_command`.

The call chain looks like this:

```
Predbat (AppDaemon)
  → sets input_boolean/input_number helpers
    → HA automation triggers
      → shell_command calls fronius_battery_control.py
        → script writes Modbus registers to inverter
```

### Why not use HA's built-in Modbus integration?

Home Assistant's native `modbus.write_register` service uses an async fire-and-forget pattern that reports success even when the write fails at the pymodbus level. In practice, writes to the Fronius GEN24 via the HA Modbus integration silently fail. This script uses a direct TCP socket and verifies each write by reading back the register value.

### Keep-alive

The revert timer means control writes must be refreshed periodically. When used with Predbat, a Home Assistant automation runs every 2 minutes to repeat the current mode's write — keeping the inverter in the requested state for as long as Predbat wants it there. The revert timer is set to 900 seconds (15 minutes) as an additional safety net.

For full setup instructions including the required HA helpers, automations and `apps.yaml` configuration, see the [Fronius section of the Predbat inverter setup documentation](https://springfall2008.github.io/batpred/inverter-setup/).

---

## Requirements

- Python 3.7+
- No external packages

---

## Contributing

Issues and PRs welcome. If you've tested this on a different Fronius model or firmware version, please open an issue to let us know what worked (or didn't). Register maps and Modbus behaviour can vary across firmware revisions.

---

## Licence

MIT
