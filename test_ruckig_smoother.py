"""Standalone check of the serving-server RuckigSmoother on real model output.

Reproduces exactly what LeRobotSystemNode._run_once does each control tick
(smoother.step(action) at 1/fps), using the cached episode-0 model predictions.
No ROS required.
"""
import os, sys
import numpy as np

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "system_Teleop/src/System_/lerobot_system")
sys.path.insert(0, PKG)
from lerobot_system.ruckig_smoother import RuckigSmoother

FPS = 30
# same defaults as the launch file: arm = HDR35_20 limits, hand dims pad with last entry
VMAX = [3.141, 3.141, 3.316, 5.410, 5.410, 7.330, 3.0]
AMAX = [12.0]
JMAX = [500.0]
ARM = slice(0, 6)


def vaj(x, dt):
    v = np.gradient(x, dt, axis=0); a = np.gradient(v, dt, axis=0); j = np.gradient(a, dt, axis=0)
    return v, a, j


def main():
    cache = ".cache_ep0_gt_pred.npz"
    if not os.path.exists(cache):
        raise SystemExit(f"missing {cache}; run compare_model_ruckig.py first")
    z = np.load(cache); pred = z["pred"]
    keep = ~np.all(z["gt"][:, 0:6] == 0, axis=1); pred = pred[keep]
    dof = pred.shape[1]
    print(f"model output: {pred.shape}, dof={dof}, fps={FPS}")

    sm = RuckigSmoother(dof=dof, control_dt=1.0 / FPS,
                        max_velocity=VMAX, max_acceleration=AMAX, max_jerk=JMAX)
    print(f"smoother limits  vmax[:6]={np.round(sm.vmax[:6],2)}  amax0={sm.amax[0]}  jmax0={sm.jmax[0]}")

    cmd = np.array([sm.step(pred[i]) for i in range(len(pred))])  # one tick per frame, exactly like the node

    _, ra, rj = vaj(pred[:, ARM], 1.0 / FPS)   # raw model output executed verbatim
    _, ca, cj = vaj(cmd[:, ARM], 1.0 / FPS)    # smoothed command stream
    amax, jmax = sm.amax[:6].max(), sm.jmax[:6].max()
    print("\narm peak |accel| (rad/s^2):  raw=%.1f  ruckig=%.1f  (limit %.0f)" % (
        np.abs(ra).max(), np.abs(ca).max(), amax))
    print("arm peak |jerk|  (rad/s^3):  raw=%.0f  ruckig=%.0f  (limit %.0f)" % (
        np.abs(rj).max(), np.abs(cj).max(), jmax))
    print("tracking error cmd-vs-raw (rad): mean=%.4f  max=%.4f" % (
        np.abs(cmd[:, ARM] - pred[:, ARM]).mean(), np.abs(cmd[:, ARM] - pred[:, ARM]).max()))

    # constant/padding hand dims must remain untouched
    pad = list(range(12, 26))
    pad_drift = np.abs(cmd[:, pad] - pred[:, pad]).max() if pad else 0.0
    print("padding-dim drift (should be ~0): %.2e" % pad_drift)

    ok = (np.abs(ca).max() <= amax * 1.1) and (np.abs(cj).max() <= jmax * 1.15)
    print("\nRESULT:", "PASS — accel & jerk within limits" if ok else "FAIL — limits exceeded")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
