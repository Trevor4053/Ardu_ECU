# Ardu_ECU Rev11 PC GUI

A desktop GUI + data logger for the ECU Rev 11 firmware. It connects to the ECU over a
USB serial port, reads the 1 Hz JSON telemetry line, sends control commands, and logs
every reading to `engine_log.csv` and `engine_log.jsonl`.

![overview]

## Requirements

* Python 3.8+
* [pyserial](https://pypi.org/project/pyserial/)  - `pip install pyserial`
* The ECU Rev 11 firmware with the serial interface (already included in `../Firmware/ECU_Rev11`)

## Running

```text
python engine_gui.py
```

1. Plug the ECU's USB port into the PC.
2. Pick the ECU's COM port from the dropdown (press **Refresh** to rescan).
3. Press **Connect**.
4. Use the buttons/sliders to control the engine.

Offline self-test (validates the parser, error bits, logging, and that every GUI command
matches the firmware grammar/ranges; no serial port needed):

```text
python engine_gui.py --selftest
```

## Controls

| Control | Sends | Effect |
| --- | --- | --- |
| START | `START` | Mode signal 100, throttle 0 -> purge |
| STOP | `STOP` | Mode signal 0 -> cooldown |
| ABORT | `ABORT` | Immediate shutdown (AbortAll) |
| RESET | `RESET` | Clear error, back to Auto Start |
| RC | `RC` | Release serial override, back to real RC/switch control |
| PING | `PING` | Liveness check |
| MODE 0-6 | `MODE n` | Force a trial mode (5 = starter only, 6 = fuel pump only) |
| Throttle slider | `THROTTLE 0-100` | Set throttle override |
| W / S | - | Keyboard: throttle +1% / -1% |
| X | - | Keyboard: cut throttle to 0% |

While connected the GUI sends a `PING` every 2 s (heartbeat) so the firmware's serial-override
watchdog (5 s, `serialCmdTimeout`) never disengages, even when no setting is being changed.

## Displays

* **RPM dial** - 0-120000, yellow arc 100000-110000, red arc 110000-120000 (ticks in thousands)
* **EGT dial** - 0-1100 C, yellow arc 630-750, red arc 750-1100
* **Throttle / Voltage** progress bars
* **Valve icons** for the gas and fuel solenoids (from the `gas` / `fuelv` telemetry fields)
* **Event log** with the CSV / JSONL file paths and `Open CSV` / `Open JSONL` / `Open folder` buttons
* **Light / Dark** theme toggle (top-left)

## Telemetry / serial interface

The firmware sends one compact JSON line per second, e.g.:

```json
{"t":12345,"mode":3,"stage":"op","thr":50,"modesig":50,"rpm":42000,"temp":610,"volt":24.3,"fuel":0,"starter":0,"glow":0,"gas":1,"fuelv":1,"err":0,"loop":2}
```

Commands are case-insensitive: `START | STOP | THROTTLE <0-100> | MODE <0-6> | SETRPM <0-100000> |
SETTEMP <0-1000> | ABORT | RESET | RC | PING | HELP`; each is acknowledged with
`CMD:<CMD> ... OK` / `CMD:<CMD> ERR` / `CMD:UNKNOWN ...`. See `ProcessSerialCommands()` /
`SendSerialTelemetry()` in the firmware.

## Logs

Each run regenerates `engine_log.csv` (one row per telemetry line) and `engine_log.jsonl`
(the raw JSON lines). The event-log panel header shows both paths with open buttons.
