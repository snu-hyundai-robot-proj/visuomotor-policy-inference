"""
Feasibility demo: apply Ruckig (jerk-limited online trajectory generation) to OUR
HDR35_20 right-arm action data.

Scenario mirrors deployment:
  - policy/RTC emits 6-DoF joint position targets at ~10 Hz
  - robot servo runs at 100 Hz
  - executing raw targets (zero-order-hold / linear) => unbounded accel & jerk
  - Ruckig streams a vel/accel/jerk-limited trajectory toward each target

Limits: velocity from HDR35_20 URDF; accel/jerk from cuRobo hdr35_20 config.
"""
import glob, os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from ruckig import Ruckig, InputParameter, OutputParameter, Result

# ---- our robot's kinematic limits (arm joints j1..j6) ----------------------
VMAX = np.array([3.141, 3.141, 3.316, 5.410, 5.410, 7.330])  # rad/s (URDF)
AMAX = np.full(6, 12.0)   # rad/s^2 (cuRobo max_acceleration)
JMAX = np.full(6, 500.0)  # rad/s^3 (cuRobo max_jerk)
DOF = 6
CTRL_DT = 0.01            # 100 Hz servo
POLICY_HZ = 10            # RTC action execution frequency (from eval_with_real_robot.py)

DATA = "/home/ngseo/hyundai_uiwang_data/converted/right/data/chunk-000/file-000.parquet"


def load_targets():
    df = pd.read_parquet(DATA, columns=["action"])
    a = np.stack(df["action"].values)[:, 0:6]          # arm joints, rad
    a = a[~np.all(a == 0, axis=1)]                      # drop zero-config frames
    step = max(1, round(30 / POLICY_HZ))               # dataset 30 Hz -> 10 Hz waypoints
    return a[::step]


def run_ruckig(wp, policy_dt):
    """Stream waypoints through Ruckig at CTRL_DT; return executed pos/vel/acc."""
    otg = Ruckig(DOF, CTRL_DT)
    inp, out = InputParameter(DOF), OutputParameter(DOF)
    inp.current_position = wp[0].tolist()
    inp.current_velocity = [0.0] * DOF
    inp.current_acceleration = [0.0] * DOF
    inp.max_velocity = VMAX.tolist()
    inp.max_acceleration = AMAX.tolist()
    inp.max_jerk = JMAX.tolist()

    # smooth pass-through: target velocity ~ central difference of waypoints
    tv = np.zeros_like(wp)
    tv[1:-1] = (wp[2:] - wp[:-2]) / (2 * policy_dt)
    tv = np.clip(tv, -VMAX, VMAX)

    ticks = max(1, round(policy_dt / CTRL_DT))
    P, V, A = [], [], []
    for i in range(len(wp)):
        inp.target_position = wp[i].tolist()
        inp.target_velocity = (tv[i] if i < len(wp) - 1 else np.zeros(DOF)).tolist()
        inp.target_acceleration = [0.0] * DOF
        for _ in range(ticks):
            res = otg.update(inp, out)
            P.append(list(out.new_position))
            V.append(list(out.new_velocity))
            A.append(list(out.new_acceleration))
            out.pass_to_input(inp)
            if res == Result.Finished:
                pass  # keep ticking to hold for the full policy window
    return np.array(P), np.array(V), np.array(A)


def naive_profiles(wp, policy_dt):
    """Zero-order-hold and linear baselines sampled at CTRL_DT."""
    ticks = max(1, round(policy_dt / CTRL_DT))
    zoh = np.repeat(wp, ticks, axis=0)
    t_wp = np.arange(len(wp)) * policy_dt
    t_ctrl = np.arange(len(wp) * ticks) * CTRL_DT
    lin = np.stack([np.interp(t_ctrl, t_wp, wp[:, j]) for j in range(DOF)], axis=1)
    return zoh, lin


def deriv(x, dt):
    return np.gradient(x, dt, axis=0)


def main():
    wp = load_targets()
    policy_dt = 1.0 / POLICY_HZ
    print(f"waypoints: {wp.shape} @ {POLICY_HZ}Hz  ->  control @ {1/CTRL_DT:.0f}Hz")

    rk_p, rk_v, rk_a = run_ruckig(wp, policy_dt)
    rk_j = deriv(rk_a, CTRL_DT)
    zoh, lin = naive_profiles(wp, policy_dt)

    def vaj(p):
        v = deriv(p, CTRL_DT); a = deriv(v, CTRL_DT); j = deriv(a, CTRL_DT)
        return v, a, j
    zoh_v, zoh_a, zoh_j = vaj(zoh)
    lin_v, lin_a, lin_j = vaj(lin)

    # ---- peak table vs limits ----
    print("\nPeak |value| per joint (rad, rad/s, rad/s^2, rad/s^3):")
    print(f"{'joint':6s} {'method':8s} {'|vel|':>8s}/{'lim':<6s} {'|acc|':>9s}/{'lim':<5s} {'|jerk|':>11s}/{'lim':<6s}")
    for j in range(DOF):
        for name, (v, a, jk) in [("ZOH", (zoh_v, zoh_a, zoh_j)),
                                 ("linear", (lin_v, lin_a, lin_j)),
                                 ("ruckig", (rk_v, rk_a, rk_j))]:
            vp, ap, jp = np.abs(v[:, j]).max(), np.abs(a[:, j]).max(), np.abs(jk[:, j]).max()
            fv = "OK" if vp <= VMAX[j] * 1.02 else "X"
            fa = "OK" if ap <= AMAX[j] * 1.05 else "X"
            fj = "OK" if jp <= JMAX[j] * 1.10 else "X"
            print(f"j{j+1:<5d} {name:8s} {vp:8.2f}/{VMAX[j]:<6.2f} "
                  f"{ap:9.1f}/{AMAX[j]:<5.0f} {jp:11.0f}/{JMAX[j]:<6.0f}  {fv}{fa}{fj}")
        print()

    # ---- plot most active joint ----
    jsel = int(np.argmax(wp.std(axis=0)))
    t_rk = np.arange(len(rk_p)) * CTRL_DT
    t_n = np.arange(len(zoh)) * CTRL_DT
    t_wp = np.arange(len(wp)) * policy_dt
    fig, ax = plt.subplots(4, 1, figsize=(13, 12), sharex=True)
    ax[0].plot(t_wp, wp[:, jsel], "ko", ms=4, label="policy targets (10Hz)")
    ax[0].plot(t_n, zoh[:, jsel], color="tab:red", lw=1, alpha=.6, label="ZOH (naive)")
    ax[0].plot(t_n, lin[:, jsel], color="tab:orange", lw=1, alpha=.7, label="linear")
    ax[0].plot(t_rk, rk_p[:, jsel], color="tab:blue", lw=2, label="ruckig")
    ax[0].set_ylabel("position (rad)"); ax[0].legend(loc="best", fontsize=8)
    for axi, (zz, ll, rr, lim, lbl) in zip(ax[1:], [
        (zoh_v, lin_v, rk_v, VMAX[jsel], "velocity (rad/s)"),
        (zoh_a, lin_a, rk_a, AMAX[jsel], "accel (rad/s²)"),
        (zoh_j, lin_j, rk_j, JMAX[jsel], "jerk (rad/s³)"),
    ]):
        axi.plot(t_n, zz[:, jsel], color="tab:red", lw=1, alpha=.6, label="ZOH")
        axi.plot(t_n, ll[:, jsel], color="tab:orange", lw=1, alpha=.7, label="linear")
        axi.plot(t_rk, rr[:, jsel], color="tab:blue", lw=2, label="ruckig")
        axi.axhline(lim, color="g", ls="--", lw=1); axi.axhline(-lim, color="g", ls="--", lw=1, label="limit")
        axi.set_ylabel(lbl); axi.legend(loc="upper right", fontsize=7)
    # zoom y for accel/jerk so ruckig is visible despite naive spikes
    ax[2].set_ylim(-AMAX[jsel]*2, AMAX[jsel]*2)
    ax[3].set_ylim(-JMAX[jsel]*3, JMAX[jsel]*3)
    ax[-1].set_xlabel("time (s)")
    fig.suptitle(f"Ruckig applied to HDR35_20 right-arm action — joint j{jsel+1} "
                 f"(most active)\nnaive execution violates accel/jerk limits; ruckig stays bounded",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = "ruckig_demo_right_arm.png"
    fig.savefig(out, dpi=140)
    print("saved:", os.path.abspath(out), "(plotted joint j%d)" % (jsel + 1))


if __name__ == "__main__":
    main()
