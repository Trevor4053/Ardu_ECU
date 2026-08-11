"""
active_valve_sim.py - offline simulator for the v11_ActiveValve fuel solenoid logic.

Mirrors the firmware's ControlOutput() fuel-path logic and ActiveValveUpdate()
(1 Hz max switching) exactly, so the valve behavior can be verified without Wokwi
or hardware. Produces a CSV, a matplotlib plot, and a console summary with
invariant checks (max one on/off cycle per second, average-flow accuracy).

Run:  python active_valve_sim.py
"""

import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_out")
os.makedirs(OUT_DIR, exist_ok=True)
CSV_PATH = os.path.join(OUT_DIR, "active_valve_sim.csv")
PNG_PATH = os.path.join(OUT_DIR, "active_valve_sim.png")

# --- firmware constants (outMin/outMax are 0/1000 in ECU_Rev11.ino) ---
OUT_MIN = 0
OUT_MAX = 1000
PUM_P_ON = 50        # pumpOnValue: minimum pump setpoint
PERIOD = 1000        # activeValvePeriod (ms): max one on/off cycle per second
DT = 10              # simulation control step in ms

PUMP_PRIME = False   # pumpPrime: keep False for normal operation

# --- firmware state (same names as v11_ActiveValve.ino) ---
fuelFlowTarget = OUT_MIN     # commanded pump value 0-1000
fuelFlowNow = OUT_MIN        # current pump output 0-1000
fuelFlow = False             # fuel solenoid command
activeValveActive = False
activeValveDuty = 0.0
activeValveOn = False
activeValveTimeOld = 0
fuelFlowTimeOld = 0


def ActiveValveUpdate(duty, now):
    """Exact copy of v11_ActiveValve.ino ActiveValveUpdate()."""
    global activeValveOn, activeValveTimeOld
    if (now - activeValveTimeOld) >= PERIOD:
        activeValveTimeOld = now
        activeValveOn = (duty > 0.0)
    if (duty < 1.0) and ((now - activeValveTimeOld) >= int(duty * PERIOD)):
        activeValveOn = False
    return activeValveOn


def ControlOutput(desired, now):
    """Fuel-path portion of ControlOutput() from v11_ActiveValve.ino."""
    global fuelFlowTarget, fuelFlowNow, fuelFlow
    global activeValveActive, activeValveDuty, activeValveOn, activeValveTimeOld

    fuelFlowTarget = desired  # control logic writes the commanded target each loop
    valveDesiredFlow = fuelFlowTarget  # captured before slew/floor clamps modify it

    # Check Limits and stop runaway increment or decrement condition
    if fuelFlowTarget > OUT_MIN:
        if fuelFlowTarget > (fuelFlowNow + 1):
            fuelFlowTarget = fuelFlowNow + 1
        if fuelFlowTarget < (fuelFlowNow - 1):
            fuelFlowTarget = fuelFlowNow - 1

    if fuelFlowTarget > OUT_MAX:
        fuelFlowTarget = OUT_MAX

    if PUM_P_ON > OUT_MIN:
        if fuelFlowTarget < PUM_P_ON:
            fuelFlowTarget = PUM_P_ON - 1
        if fuelFlowNow < PUM_P_ON:
            fuelFlowNow = PUM_P_ON - 1
    elif fuelFlowTarget < OUT_MIN:
        fuelFlowTarget = OUT_MIN

    # Active Valve: commanded flow below pump minimum -> pulse the solenoid
    if (not PUMP_PRIME) and (PUM_P_ON > OUT_MIN) and (valveDesiredFlow < PUM_P_ON):
        if valveDesiredFlow > OUT_MIN:
            # start pulsing with the valve open (mirrors firmware fix)
            if not activeValveActive:
                activeValveOn = True
                activeValveTimeOld = now
            activeValveActive = True
            activeValveDuty = float(valveDesiredFlow) / float(PUM_P_ON)
            if activeValveDuty < 0.0:
                activeValveDuty = 0.0
            if activeValveDuty > 1.0:
                activeValveDuty = 1.0
        else:
            activeValveActive = False
            activeValveDuty = 0.0
    else:
        activeValveActive = False
        activeValveDuty = 0.0

    # Fuel Flow Solenoid control
    if (not PUMP_PRIME) and (PUM_P_ON > OUT_MIN) and (fuelFlowTarget > PUM_P_ON):
        fuelFlow = True
    elif (not PUMP_PRIME) and (PUM_P_ON == OUT_MIN) and (fuelFlowTarget > OUT_MIN):
        fuelFlow = True
    elif activeValveActive:
        fuelFlow = ActiveValveUpdate(activeValveDuty, now)
    else:
        fuelFlow = False

    # pump output ramp (simplified: 1 unit per control step)
    if fuelFlowNow < fuelFlowTarget:
        fuelFlowNow += 1
    elif fuelFlowNow > fuelFlowTarget:
        fuelFlowNow -= 1


def profile(t):
    """Commanded fuel flow (fuelFlowTarget written by the control logic)."""
    if t < 4000:
        return 40            # below pumpOnValue(50) -> 80% duty pulsed (steady)
    if t < 8000:
        return 25            # below pumpOnValue -> 50% duty pulsed (steady)
    if t < 12000:
        return 0             # fuel cut -> solenoid off, pump at floor
    if t < 16000:
        return 500           # normal flow -> solenoid continuous ON
    return 1000              # high operating flow -> continuous ON


def transition_check():
    """Re-engagement after a fuel cut must open the valve immediately.

    With a stale pulse phase, re-entering the pulsed regime could hold the valve
    closed for up to one full second (fuel starvation on rapid throttle dips).
    The firmware fix starts each pulse regime with the valve OPEN.
    """
    global fuelFlowTarget, fuelFlowNow, fuelFlow
    global activeValveActive, activeValveDuty, activeValveOn, activeValveTimeOld

    fuelFlowTarget = fuelFlowNow = 0
    fuelFlow = False
    activeValveActive = False
    activeValveDuty = 0.0
    activeValveOn = False
    activeValveTimeOld = 0

    # engage at boot; brief 150 ms cut at t=750 (mid pulse phase); re-engage at 900.
    # Without the firmware fix, a stale pulse phase holds the valve closed through
    # the re-engagement (starvation window up to 1 s).
    profile = [(0, 750, 25), (750, 900, 0), (900, 3000, 25)]
    eng = {}       # engage time -> valve state at that instant
    first_off = {} # engage time -> time of first OFF after engagement
    prev = None
    for t in range(0, 3000, DT):
        desired = next(v for (t0, t1, v) in profile if t0 <= t < t1)
        was_active = activeValveActive
        ControlOutput(desired, t)
        if activeValveActive and not was_active:
            eng[t] = fuelFlow          # valve state at the engage step
            first_off[t] = None
        for e in eng:
            if first_off[e] is None and prev is True and fuelFlow is False and e < t:
                first_off[e] = t
        prev = fuelFlow

    ok = True
    for e in sorted(eng):
        if eng[e] is not True:
            ok = False
        print(f"  engage at t={e} ms: valve={'ON' if eng[e] else 'OFF'}"
              f"  first OFF={first_off[e]}")
    open_ok = all(v is True for v in eng.values())
    print(f"transition: {'PASS - valve opens immediately on (re)engagement' if open_ok else 'FAIL - valve started closed'}")
    return open_ok


def main():
    rows = []
    switches = 0
    prev_valve = None
    # count transitions per 1 s window to enforce the "once per second" rule
    window_t = {}
    max_per_window = 0

    T_END = 19000
    for t in range(0, T_END + DT, DT):
        ControlOutput(profile(t), t)
        rows.append((t, profile(t), fuelFlowTarget, fuelFlowNow,
                     int(fuelFlow), activeValveDuty))
        if prev_valve is not None and fuelFlow != prev_valve:
            switches += 1
            win = t // 1000
            window_t[win] = window_t.get(win, 0) + 1
        prev_valve = fuelFlow
    max_per_window = max(window_t.values(), default=0)

    # write CSV
    csv_path = CSV_PATH
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "desired", "target_clamped", "pump_out",
                    "solenoid", "duty"])
        w.writerows(rows)

    # average-flow accuracy in steady pulsed windows (settled after ~1 s)
    win80 = [r for r in rows if 1000 <= r[0] < 4000]   # desired 40 -> 80% duty
    win50 = [r for r in rows if 5000 <= r[0] < 8000]   # desired 25 -> 50% duty
    f80 = sum(r[4] for r in win80) / len(win80)
    f50 = sum(r[4] for r in win50) / len(win50)

    # plot
    ts = [r[0] for r in rows]
    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, sharex=True, figsize=(10, 7), constrained_layout=True)
    ax1.plot(ts, [r[1] for r in rows], label="desired flow", color="tab:blue")
    ax1.plot(ts, [r[2] for r in rows], label="target (after clamps)",
             color="tab:orange", linestyle="--")
    ax1.plot(ts, [r[3] for r in rows], label="pump output", color="tab:green")
    ax1.axhline(PUM_P_ON, color="red", linestyle=":", label="pumpOnValue (50)")
    ax1.set_ylabel("flow (0-1000)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3)
    ax2.step(ts, [r[4] for r in rows], where="post", color="tab:purple")
    ax2.set_ylabel("solenoid (0/1)")
    ax2.set_ylim(-0.1, 1.1)
    ax2.grid(alpha=0.3)
    ax3.plot(ts, [r[5] for r in rows], color="tab:red")
    ax3.set_ylabel("valve duty")
    ax3.set_xlabel("time (ms)")
    ax3.set_ylim(0, 1.05)
    ax3.grid(alpha=0.3)
    png_path = PNG_PATH
    fig.savefig(png_path, dpi=110)

    # console summary
    print(f"pumpOnValue = {PUM_P_ON}   activeValvePeriod = {PERIOD} ms   step = {DT} ms")
    print(f"total solenoid on/off transitions: {switches}")
    print(f"max transitions in any 1 s window: {max_per_window}"
          f"  (one cycle/sec allows up to 2)")
    print(f"steady 80%% duty (1-4 s): on-time = {f80:.3f} -> avg flow = {f80*PUM_P_ON:.1f}"
          f" (desired 40)")
    print(f"steady 50%% duty (5-8 s): on-time = {f50:.3f} -> avg flow = {f50*PUM_P_ON:.1f}"
          f" (desired 25)")
    print(f"compliance: {'PASS' if max_per_window <= 2 else 'FAIL'}"
          f" max one on/off cycle per second")
    print()
    transition_check()
    print("note: if a low flow is commanded while the pump output is still high, the")
    print("      solenoid stays continuous ON during the pump's slew-limited decay,")
    print("      then switches to pulsing once the clamped target falls below pumpOnValue")
    print(f"CSV: {csv_path}")
    print(f"Plot: {png_path}")

    # ASCII timeline of the solenoid over the whole run (200 ms cells)
    print("\nsolenoid timeline (each cell = 200 ms, . = off, # = on):")
    line = ""
    for r in rows:
        if r[0] % 200 == 0:
            line += "#" if r[4] else "."
    print(line)
    print("      0        4s        8s        12s       16s")


if __name__ == "__main__":
    main()


