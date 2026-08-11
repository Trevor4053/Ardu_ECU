# v11_ActiveValve

A fork of the ECU Rev 11 firmware that adds **active (pulsed) fuel solenoid control**
so the fuel pump is never forced to run below its minimum setpoint.

## Feature

The stock Rev 11 firmware drives the fuel pump with a proportional output
(`fuelFlowTarget` / `fuelFlowNow`, mapped to microseconds on `fuelServo`) and uses the
fuel solenoid (`Fuel_Solenoid_Pin`, GPIO 16) only as a digital on/off valve. The pump has a
minimum setpoint (`pumpOnValue`): when the commanded fuel flow drops below it, the pump
cannot go any slower and the only options are full minimum flow or zero flow.

This fork adds a third regime:

| Commanded flow | Pump | Fuel solenoid |
| --- | --- | --- |
| `fuelFlowTarget >= pumpOnValue` | normal proportional control | ON (continuous) |
| `0 < fuelFlowTarget < pumpOnValue` | held at minimum setpoint | pulsed at up to 1 Hz |
| `fuelFlowTarget == 0` | minimum setpoint | OFF |

(The `>=` boundary is deliberate: at exactly `pumpOnValue` the solenoid stays on - the
stock `>` logic cut the fuel dead at that single point, which the idle governor hunted
across.)

In the pulsed regime the solenoid duty cycle is

```
duty = fuelFlowTarget / pumpOnValue
```

so the *average* fuel flow reaching the engine matches the commanded flow, down to a few
percent of minimum. The pulse period is fixed at `activeValvePeriod = 1000 ms`. The
solenoid never switches between on and off more than once per second, i.e. the maximum
pulse frequency is 1 Hz (`activeValvePeriod` is a `#define` at the top of
`v11_ActiveValve.ino`; a longer period is allowed, a shorter one is not).

### Notes

- The pump still runs at its existing minimum floor (`pumpOnValue - 1`, unchanged from
  stock Rev 11), so pump handling is identical outside the pulsed regime.
- Every entry into the pulsed regime starts with the solenoid **open** (the pulse phase
  is re-initialized), so a momentary fuel cut followed by re-engagement can never leave
  the valve closed for up to a second - no fuel-starvation gap.
- While the solenoid is pulsed, the serial telemetry `fuelv` field reports the live
  solenoid state (`1` during the on-phase, `0` during the off-phase).
- Pump-prime mode and the solenoid on/off logic are otherwise unchanged; the pulsed
  regime only engages when `!pumpPrime && pumpOnValue > outMin && 0 < target < pumpOnValue`.
- **Sub-minimum flow works in the running modes too.** In stock, the idle/operating fuel
  governor is a +/-1 integrator on `fuelFlowTarget` and `ControlOutput()` floored that
  value at `pumpOnValue - 1`, so the flow could never be commanded below the pump
  minimum (and at the exact floor the stock `>` solenoid logic cut the fuel). In this
  fork the pump-minimum floor applies to the pump output only, the integrator is free to
  command below `pumpOnValue`, and the solenoid duty carries the reduction. The
  anti-runaway down-clamp and the pump deceleration branches are likewise floored at the
  pump minimum so the pump output never waggles in the pulsed regime.
- The idle/operating "hold" path no longer snaps `fuelFlowTarget` to `fuelFlowNow`
  (snapping would pin the target to the pump floor and defeat the pulsing); it now
  simply holds the current target.

## Build

Same board and toolchain as stock Rev 11 (ESP32-S3, `esp32:esp32:esp32s3`):

```
arduino-cli compile --fqbn esp32:esp32:esp32s3 "v11_ActiveValve"
```

## Simulation (no hardware / no Wokwi)

`active_valve_sim.py` models the fuel-path of `ControlOutput()` and the exact
`ActiveValveUpdate()` logic from the firmware, so the valve behavior can be verified
offline:

```
python active_valve_sim.py
```

It runs a commanded-flow scenario (pulsed below the pump minimum, fuel cut, then normal
operation), checks that the solenoid never completes more than one on/off cycle per second,
and verifies the delivered average flow matches the command (e.g. 80% duty -> 40 of a 50
`pumpOnValue`). Outputs a console summary plus `sim_out/active_valve_sim.csv` and
`sim_out/active_valve_sim.png` (requires `pyserial`-independent `matplotlib` and `numpy`).

The sim also checks the re-engagement transition (valve must open immediately) and the
idle-governor walk-down: emulating the +/-1 integrator of `IdlingFunction`/`OperatingFunction`,
the target and pulse duty must be able to fall below `pumpOnValue` while the pump stays at
its floor. Both checks fail against the pre-fix logic and pass now.

The sim also reproduces a real stock behavior worth knowing: if a low flow is commanded
while the pump output is still high, the solenoid stays continuously ON during the pump's
slew-limited decay and only starts pulsing once the clamped target falls below `pumpOnValue`.

## Differences from stock ECU_Rev11

- `#define activeValvePeriod` + Active Valve globals (`activeValveActive`,
  `activeValveDuty`, `activeValveOn`, `activeValveTimeOld`)
- `ActiveValveUpdate(duty)` - 1 Hz on/off pulse generator for the solenoid
- `ControlOutput()` captures the true commanded flow before the slew/floor clamps and
  selects the pulsed regime when flow is below the pump minimum; the pump-minimum floor
  and the anti-runaway down-clamp apply to the pump output only, and the continuous
  solenoid branch uses `>= pumpOnValue` (no dead-zone fuel cut)
- `IdlingFunction()` / `OperatingFunction()` hold the fuel target instead of snapping it
  to the pump output, so the governor can command below the pump minimum
- Cross-core logging safety: the LittleFS header is written by the core-0 logging task
  and error messages are passed core 1 -> core 0 as `volatile` flags (`sysMsgPending` /
  `sysMsgErrCode`) instead of a `String` shared between the two cores
