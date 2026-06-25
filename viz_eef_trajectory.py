"""
FK-based EEF (flange/tool0) 3D trajectory visualization for the HDR35_20 arm.

- Parses joint origins/axes directly from the hdr35_20 xacro (valid XML; meshes ignored).
- Computes forward kinematics in numpy from arm joints (action[:, 0:6] = j1..j6).
- Plots the end-effector (tool0) 3D trajectory per episode and saves a PNG.

Usage:
    python viz_eef_trajectory.py --side right --source action
"""
import argparse
import glob
import os
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

URDF_XACRO = (
    "system_Teleop/src/Robot_/src/hdr_description/"
    "urdf/robots/hdr35_20/hdr35_20.urdf.xacro"
)
ARM_JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]  # maps to action[:, 0:6]
EEF_LINK = "tool0"
BASE_LINK = "base_link"


# ----- basic SE(3) helpers --------------------------------------------------
def rpy_to_matrix(r, p, y):
    """URDF rpy -> R = Rz(y) @ Ry(p) @ Rx(r) (extrinsic xyz)."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_T(axis, theta):
    """Rotation about (unit) axis by theta as a 4x4 transform (Rodrigues)."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s, C = np.cos(theta), np.sin(theta), 1 - np.cos(theta)
    R = np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    return T


# ----- parse the (xacro) URDF kinematic tree --------------------------------
def parse_joints(path):
    """Return child_link -> dict(parent, origin T, axis, type, name)."""
    tree = ET.parse(path)
    root = tree.getroot()
    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        jtype = j.get("type")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
        ax = j.find("axis")
        axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1]
        joints[child] = dict(parent=parent, T=make_T(xyz, rpy), axis=axis,
                             type=jtype, name=name)
    return joints


def chain_to(joints, target, base):
    """Ordered list of joint entries from base down to target link."""
    chain = []
    link = target
    while link != base:
        if link not in joints:
            raise RuntimeError(f"Link {link} has no parent joint; cannot reach {base}")
        jt = joints[link]
        chain.append(jt)
        link = jt["parent"]
    return list(reversed(chain))


def fk_positions(chain, q_by_name):
    """q_by_name: (N, ) arrays keyed by joint name. Returns (N,3) eef positions."""
    n = next(iter(q_by_name.values())).shape[0] if q_by_name else 1
    T = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
    for jt in chain:
        Torigin = jt["T"]
        T = T @ Torigin
        if jt["type"] in ("revolute", "continuous"):
            q = q_by_name[jt["name"]]
            Tj = np.stack([axis_angle_T(jt["axis"], qi) for qi in q])
            T = T @ Tj
    return T[:, :3, 3]


# ----- main -----------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="right", choices=["left", "right"])
    ap.add_argument("--source", default="action", choices=["action", "observation.state"])
    ap.add_argument("--data_root", default="/home/ngseo/hyundai_uiwang_data/converted")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    joints = parse_joints(URDF_XACRO)
    chain = chain_to(joints, EEF_LINK, BASE_LINK)
    print("FK chain:", " -> ".join([f"{j['name']}({j['type']})" for j in chain]))

    files = sorted(glob.glob(os.path.join(args.data_root, args.side,
                                           "data", "chunk-*", "file-*.parquet")))
    print(f"{len(files)} episode files for side={args.side}")

    episodes = []
    for f in files:
        df = pd.read_parquet(f, columns=[args.source])
        arr = np.stack(df[args.source].values)[:, 0:6]  # arm joints j1..j6
        # drop all-zero arm frames (recording artifact, e.g. frame 0 pre-start)
        keep = ~np.all(arr == 0, axis=1)
        dropped = int((~keep).sum())
        if dropped:
            print(f"  {os.path.basename(f)}: dropped {dropped} zero-arm frame(s)")
        arr = arr[keep]
        q = {name: arr[:, i] for i, name in enumerate(ARM_JOINTS)}
        pos = fk_positions(chain, q)
        episodes.append((os.path.basename(f), pos))

    allpos = np.concatenate([p for _, p in episodes])
    print("EEF xyz range (m): "
          f"x[{allpos[:,0].min():.3f},{allpos[:,0].max():.3f}] "
          f"y[{allpos[:,1].min():.3f},{allpos[:,1].max():.3f}] "
          f"z[{allpos[:,2].min():.3f},{allpos[:,2].max():.3f}]")

    cmap = plt.get_cmap("turbo", len(episodes))
    fig = plt.figure(figsize=(16, 12))
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    projviews = [("XY", 0, 1), ("XZ", 0, 2), ("YZ", 1, 2)]
    axp = [fig.add_subplot(2, 2, k) for k in (2, 3, 4)]

    for idx, (name, pos) in enumerate(episodes):
        c = cmap(idx)
        ep = name.replace("file-", "ep").replace(".parquet", "")
        ax3d.plot(pos[:, 0], pos[:, 1], pos[:, 2], color=c, lw=1.2, alpha=0.9, label=ep)
        ax3d.scatter(*pos[0], color=c, marker="o", s=30)   # start
        ax3d.scatter(*pos[-1], color=c, marker="x", s=40)  # end
        for (title, a, b), axx in zip(projviews, axp):
            axx.plot(pos[:, a], pos[:, b], color=c, lw=1.0, alpha=0.85)
            axx.scatter(pos[0, a], pos[0, b], color=c, marker="o", s=20)
            axx.scatter(pos[-1, a], pos[-1, b], color=c, marker="x", s=25)

    ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")
    ax3d.set_title(f"EEF (tool0) 3D trajectory — {args.side} | source={args.source}")
    ax3d.legend(fontsize=7, ncol=2, loc="upper left")
    for (title, a, b), axx in zip(projviews, axp):
        axx.set_title(f"{title} projection")
        axx.set_xlabel(f"{'XYZ'[a]} (m)"); axx.set_ylabel(f"{'XYZ'[b]} (m)")
        axx.axis("equal"); axx.grid(True, alpha=0.3)
    fig.suptitle(f"HDR35_20 forward-kinematics EEF trajectory ({args.side} arm, "
                 f"{len(episodes)} episodes)", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    out = args.out or f"eef_trajectory_{args.side}_{args.source.split('.')[-1]}.png"
    fig.savefig(out, dpi=140)
    print("saved:", os.path.abspath(out))


if __name__ == "__main__":
    main()
