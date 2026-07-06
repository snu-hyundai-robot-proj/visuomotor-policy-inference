#!/usr/bin/env python3
"""Simplified per-joint control GUI for the DG5F LEFT hand — 12 active DOF, normalized 0..1.

Only the flexion joints of the 4 non-thumb fingers are exposed: joints _2/_3/_4 of index,
middle, ring, pinky = 12 sliders. Each slider is normalized 0 (open) .. 1 (closed), mapped to
that joint's recorded open->grip range. The THUMB (all 4 joints) and each finger's _1 joint are
HELD FIXED at their init (open) values. Init = 0 (fully open).

Each slider shows its normalized value AND the actual joint value (rad) read from joint_states.
Buttons: Open (all -> 0) / Close (all -> 1) / Sync (leave fixed, zero the sliders).

  (run in a teleop container; see hand_joint_gui.sh)
"""
from __future__ import annotations

import sys
import threading
import time

import numpy as np

import rclpy                                          # noqa: E402
from rclpy.node import Node                           # noqa: E402
from control_msgs.msg import MultiDOFCommand          # noqa: E402
from sensor_msgs.msg import JointState                # noqa: E402

from PyQt5 import QtCore, QtGui, QtWidgets            # noqa: E402

SEND_DT = 0.01           # 100 Hz command stream
NAMES = [f"lj_dg_{f}_{j}" for f in range(1, 6) for j in range(1, 5)]

# init pose (all 20 joints) — fixed joints stay here; active joints start here (norm 0 = open).
INIT_POSE = np.array([-0.16, 1.104, -0.207, 0.0, -0.349, 0.0, 0.519, 0.414, -0.384, 0.317,
                      0.224, 0.178, -0.384, 0.641, 0.0, 0.0, 0.064, -0.419, 0.721, 0.0])
# recorded grip pose — active joints reach here at norm 1 = closed.
GRIP_POSE = np.array([-0.482, 1.104, -1.152, -1.599, -0.349, 1.245, 2.666, 1.92, -0.384, 1.576,
                      2.747, 1.92, -0.384, 1.632, 2.241, 1.855, 0.301, -0.419, 1.801, 1.551])

# 12 ACTIVE joints: joints _2/_3/_4 of the 4 non-thumb fingers (index/middle/ring/pinky).
FINGER_LABEL = {2: "Index (2)", 3: "Middle (3)", 4: "Ring (4)", 5: "Pinky (5)"}
ACTIVE = []          # list of (finger, joint, idx, open_val, grip_val)
for f in range(2, 6):
    for j in range(2, 5):
        idx = (f - 1) * 4 + (j - 1)
        ACTIVE.append((f, j, idx, float(INIT_POSE[idx]), float(GRIP_POSE[idx])))
ACTIVE_IDX = [a[2] for a in ACTIVE]

QSS = """
QWidget { background:#1e1e2e; color:#cdd6f4; font-family:'DejaVu Sans'; font-size:12px; }
QGroupBox { border:1px solid #45475a; border-radius:8px; margin-top:12px; padding:8px; font-weight:bold; }
QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 4px; color:#89b4fa; }
QPushButton { background:#313244; border:1px solid #45475a; border-radius:6px; padding:6px 12px; }
QPushButton:hover { background:#585b70; }
QLabel#nrm { color:#a6e3a1; font-family:monospace; }
QLabel#act { color:#f9e2af; font-family:monospace; }
QLabel#jn  { color:#bac2de; font-family:monospace; }
QSlider::groove:horizontal { height:6px; background:#45475a; border-radius:3px; }
QSlider::handle:horizontal { background:#89b4fa; width:16px; border-radius:8px; margin:-6px 0; }
QSlider::sub-page:horizontal { background:#f38ba8; border-radius:3px; }
"""


class HandNode(Node):
    def __init__(self):
        super().__init__("hand_joint_gui")
        self.pub = self.create_publisher(MultiDOFCommand, "/dg5f_left/lj_dg_pospid/reference", 1)
        self.actual = np.full(20, np.nan)
        self.create_subscription(JointState, "/dg5f_left/joint_states", self._js, 10)

    def _js(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        for k, n in enumerate(NAMES):
            if n in idx:
                self.actual[k] = msg.position[idx[n]]

    def publish(self, vals):
        self.pub.publish(MultiDOFCommand(dof_names=NAMES, values=[float(x) for x in vals],
                                         values_dot=[0.0] * 20))


class Streamer(threading.Thread):
    """Streams the full 20-joint command: fixed joints at INIT, active from normalized 0..1."""
    def __init__(self, node):
        super().__init__(daemon=True)
        self.node = node
        self.lock = threading.Lock()
        self.norm = np.zeros(len(ACTIVE))     # 0..1 per active joint (0 = open)
        self.running = True

    def set_norm(self, k, v):
        with self.lock: self.norm[k] = float(np.clip(v, 0.0, 1.0))

    def set_all(self, v):
        with self.lock: self.norm[:] = float(np.clip(v, 0.0, 1.0))

    def get_norm(self):
        with self.lock: return self.norm.copy()

    def _pose(self, norm):
        out = INIT_POSE.copy()
        for k, (f, j, idx, o, g) in enumerate(ACTIVE):
            out[idx] = o + norm[k] * (g - o)
        return out

    def run(self):
        while self.running:
            self.node.publish(self._pose(self.get_norm()))
            time.sleep(SEND_DT)


class Window(QtWidgets.QMainWindow):
    def __init__(self, node, streamer):
        super().__init__()
        self.node, self.st = node, streamer
        self.setWindowTitle("DG5F LEFT — 12 DOF (fingers _2/_3/_4, normalized 0..1)")
        self.setStyleSheet(QSS)
        self.rows = []          # (k, idx, slider, nrm_label, act_label)
        self._build()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self._refresh); self.timer.start(100)

    def _build(self):
        cw = QtWidgets.QWidget(); root = QtWidgets.QVBoxLayout(cw)
        note = QtWidgets.QLabel("thumb + each finger's _1 joint are FIXED (init). 12 sliders = "
                                "index/middle/ring/pinky × _2/_3/_4,  0 = open  →  1 = closed.")
        note.setStyleSheet("color:#f9e2af;"); root.addWidget(note)
        bar = QtWidgets.QHBoxLayout()
        for label, fn in [("Open (all 0)", lambda: self._all(0.0)),
                          ("Close (all 1)", lambda: self._all(1.0))]:
            b = QtWidgets.QPushButton(label); b.setFocusPolicy(QtCore.Qt.NoFocus)
            b.clicked.connect(fn); bar.addWidget(b)
        bar.addStretch(1); root.addLayout(bar)

        cols = QtWidgets.QHBoxLayout(); root.addLayout(cols)
        by_finger = {2: [], 3: [], 4: [], 5: []}
        for k, (f, j, idx, o, g) in enumerate(ACTIVE):
            by_finger[f].append((k, j, idx, o, g))
        for f in (2, 3, 4, 5):
            gb = QtWidgets.QGroupBox(FINGER_LABEL[f]); gl = QtWidgets.QGridLayout(gb)
            gl.addWidget(QtWidgets.QLabel("joint"), 0, 0)
            gl.addWidget(QtWidgets.QLabel("norm"), 0, 2)
            gl.addWidget(QtWidgets.QLabel("act(rad)"), 0, 3)
            for r, (k, j, idx, o, g) in enumerate(by_finger[f]):
                jn = QtWidgets.QLabel(f"_{j}"); jn.setObjectName("jn")
                s = QtWidgets.QSlider(QtCore.Qt.Horizontal); s.setFocusPolicy(QtCore.Qt.NoFocus)
                s.setRange(0, 100); s.setValue(0); s.setMinimumWidth(150)
                s.valueChanged.connect(lambda v, kk=k: self.st.set_norm(kk, v / 100.0))
                nl = QtWidgets.QLabel("0.00"); nl.setObjectName("nrm")
                al = QtWidgets.QLabel("  ? "); al.setObjectName("act")
                deg = " (fixed range)" if abs(g - o) < 1e-3 else ""
                gl.addWidget(jn, r + 1, 0); gl.addWidget(s, r + 1, 1)
                gl.addWidget(nl, r + 1, 2); gl.addWidget(al, r + 1, 3)
                if deg:
                    jn.setText(f"_{j}·")
                self.rows.append((k, idx, s, nl, al))
            cols.addWidget(gb)
        hint = QtWidgets.QLabel("green = normalized command (0..1)   yellow = actual joint (rad)   "
                                "·= no motion range in the recorded data")
        hint.setStyleSheet("color:#7f849c;"); root.addWidget(hint)
        self.setCentralWidget(cw)

    def _all(self, v):
        self.st.set_all(v)
        for k, idx, s, nl, al in self.rows:
            s.blockSignals(True); s.setValue(int(v * 100)); s.blockSignals(False)

    def _refresh(self):
        norm = self.st.get_norm(); act = self.node.actual
        for k, idx, s, nl, al in self.rows:
            nl.setText(f"{norm[k]:.2f}")
            al.setText("  ? " if np.isnan(act[idx]) else f"{act[idx]:+.2f}")

    def closeEvent(self, ev):
        self.st.running = False; time.sleep(0.05); ev.accept()


def main():
    rclpy.init()
    node = HandNode()
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True); spin.start()
    time.sleep(0.3)
    st = Streamer(node); st.start()

    app = QtWidgets.QApplication(sys.argv)
    win = Window(node, st); win.resize(1080, 340); win.show()
    win.raise_(); win.activateWindow()
    code = app.exec_()
    st.running = False
    node.destroy_node(); rclpy.shutdown(); sys.exit(code)


if __name__ == "__main__":
    main()
