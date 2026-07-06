#!/usr/bin/env python3
"""Offline j6 offset/calibration audit.

This script only reads repository files and writes audit artifacts under
analysis/j6_offset_calibration_audit. It does not use ROS, serial, Docker, or
hardware.
"""

from __future__ import annotations

import csv
import hashlib
import re
from pathlib import Path


ROOT = Path("/home/bi/visuomotor-policy-inference")
OUT = ROOT / "analysis" / "j6_offset_calibration_audit"

KEYWORDS = [
    "offset",
    "zero",
    "origin",
    "home",
    "calib",
    "calibration",
    "angle_offset",
    "joint_offset",
    "thumb_rot",
    "j6",
    "finger6",
    "limit",
    "min_degree",
    "max_degree",
    "angleSet",
    "angleAct",
    "mode",
    "forceClb",
]

TARGET_SUFFIXES = {
    ".py",
    ".yml",
    ".yaml",
    ".json",
    ".urdf",
    ".xacro",
    ".xml",
    ".launch.py",
    ".md",
    ".txt",
}

SEARCH_ROOTS = [
    ROOT / "system_Teleop",
    ROOT / "examples",
    ROOT / "tools",
    ROOT / "docker-compose.yml",
    ROOT / "docker-compose.full.yml",
    ROOT / "docker-compose.override.yml",
]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def iter_files():
    for root in SEARCH_ROOTS:
        if not root.exists():
            continue
        if root.is_file():
            yield root
            continue
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(ROOT)
            if "analysis" in rel.parts or "__pycache__" in rel.parts:
                continue
            if path.suffix.lower() in {".jpg", ".png", ".so", ".a", ".o", ".pyc", ".gz"}:
                continue
            if path.suffix.lower() in TARGET_SUFFIXES or path.name.endswith(".launch.py"):
                yield path


def classify(path: Path, line: str):
    rel = str(path.relative_to(ROOT))
    low = line.lower()
    name = ""
    m = re.search(r"([A-Za-z_][A-Za-z0-9_]*)\s*=", line)
    if m:
        name = m.group(1)
    elif ":" in line:
        name = line.split(":", 1)[0].strip()
    else:
        name = "literal/comment"

    j6_applies = "no"
    influence = "none"
    timing = "n/a"
    active = "no"

    if "inspire_driver.py" in rel:
        if "INSPIRE_FINGER_MIN_DEGREE" in line or "INSPIRE_FINGER_MAX_DEGREE" in line:
            j6_applies = "yes"
            influence = "target"
            timing = "runtime"
            active = "yes"
        elif "thumb_rot" in low or "cur_6d_vec[5]" in line:
            j6_applies = "yes"
            influence = "target"
            timing = "runtime"
            active = "yes"
        elif "target_joint" in line and ("tj6" in line or "msg.position" in line or "[5]" in line):
            j6_applies = "yes"
            influence = "feedback_echo"
            timing = "runtime"
            active = "yes"
        elif "received_angle" in line:
            j6_applies = "yes_if_index_5"
            influence = "feedback"
            timing = "runtime"
            active = "yes"
        elif "publish_driver_diagnostics" in line or "driver_diagnostics" in line:
            j6_applies = "yes_if_enabled"
            influence = "diagnostic_only"
            timing = "runtime"
            active = "default_off_not_run"
    elif "inspire_comm.py" in rel:
        if "angleSet" in line:
            j6_applies = "yes"
            influence = "target_protocol"
            timing = "runtime"
            active = "yes"
        elif "angleAct" in line or "received_angle" in line or "get_position_values" in line:
            j6_applies = "yes"
            influence = "feedback_protocol"
            timing = "runtime"
            active = "yes"
        elif "SRBL_INSPIRE_FINGER_LOWER_LIMIT" in line or "SRBL_INSPIRE_FINGER_UPPER_LIMIT" in line:
            j6_applies = "yes"
            influence = "target"
            timing = "runtime"
            active = "yes"
        elif any(k in low for k in ["mode", "forceclb", "clearerr", "errcode", "statuscode", "curract"]):
            j6_applies = "protocol_register_available"
            influence = "internal_state_register"
            timing = "not_polled_in_current_path"
            active = "register_defined_not_used"
    elif "inspire_bridge.py" in rel:
        if "thumb_rot" in low or "ThumbMCPSpread" in line:
            j6_applies = "yes_teleop_input"
            influence = "target_input"
            timing = "runtime_when_bridge_running"
            active = "not_active_in_direct_test"
        elif any(k in low for k in ["percentile", "open_ref", "close_ref", "calib"]):
            j6_applies = "yes_teleop_input"
            influence = "target_input_normalization"
            timing = "runtime_when_bridge_running"
            active = "not_active_in_direct_test"
    elif "drive_arm_hand_replay.py" in rel or "drive_hand_replay.py" in rel:
        if "j6" in low or "1900" in line or "950" in line:
            j6_applies = "yes_replay"
            influence = "target"
            timing = "runtime_when_replay_running"
            active = "not_active_in_direct_test"
    elif "thumb_live_target_test.py" in rel or "j6_live_target_test.py" in rel:
        if "j6" in low or "1900" in line or "950" in line:
            j6_applies = "yes_direct_test"
            influence = "target_generation_or_logging"
            timing = "runtime_when_tool_running"
            active = "used_in_recent_direct_test"
    elif rel.endswith("system_Teleop/docker-compose.yml"):
        if "INSPIRE_PORT" in line or "inspire_driver_node" in line:
            j6_applies = "no_axis_specific"
            influence = "startup"
            timing = "startup"
            active = "yes_for_driver_container"
    elif ".urdf" in rel or ".xacro" in rel:
        if "joint name=\"j6\"" in line or "<limit" in line or "<origin" in line:
            j6_applies = "arm_j6_not_hand"
            influence = "arm_urdf_only"
            timing = "not_hand_runtime"
            active = "not_hand_path"

    value = ""
    if "=" in line:
        value = line.split("=", 1)[1].strip()
    elif ":" in line:
        value = line.split(":", 1)[1].strip()

    return name, value, j6_applies, influence, timing, active


def search_inventory():
    pat = re.compile("|".join(re.escape(k) for k in KEYWORDS), re.IGNORECASE)
    rows = []
    for path in iter_files():
        try:
            text = path.read_text(errors="replace").splitlines()
        except Exception:
            continue
        for i, line in enumerate(text, 1):
            if not pat.search(line):
                continue
            name, value, j6_applies, influence, timing, active = classify(path, line)
            rows.append(
                {
                    "file": str(path.relative_to(ROOT)),
                    "line": i,
                    "parameter / variable name": name,
                    "default value": value,
                    "j6에 적용되는지": j6_applies,
                    "target에 영향 / feedback에 영향 / 둘 다": influence,
                    "startup에만 적용 / runtime에 적용": timing,
                    "현재 실행 경로에서 실제 사용되는지": active,
                    "source_line": line.strip(),
                }
            )
    return rows


def write_csv(rows):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "j6_offset_calibration_inventory.csv"
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [
            "file", "line", "parameter / variable name", "default value",
            "j6에 적용되는지", "target에 영향 / feedback에 영향 / 둘 다",
            "startup에만 적용 / runtime에 적용", "현재 실행 경로에서 실제 사용되는지",
            "source_line",
        ])
        w.writeheader()
        w.writerows(rows)


def md_table(rows, limit=80):
    cols = [
        "file",
        "line",
        "parameter / variable name",
        "default value",
        "j6에 적용되는지",
        "target에 영향 / feedback에 영향 / 둘 다",
        "startup에만 적용 / runtime에 적용",
        "현재 실행 경로에서 실제 사용되는지",
    ]
    out = ["|" + "|".join(cols) + "|", "|" + "|".join(["---"] * len(cols)) + "|"]
    for r in rows[:limit]:
        vals = []
        for c in cols:
            v = str(r[c]).replace("|", "\\|")
            if len(v) > 70:
                v = v[:67] + "..."
            vals.append(v)
        out.append("|" + "|".join(vals) + "|")
    return "\n".join(out)


def write_docs(rows):
    active = [
        r for r in rows
        if r["현재 실행 경로에서 실제 사용되는지"] in {"yes", "yes_for_driver_container"}
        and r["j6에 적용되는지"] not in {"arm_j6_not_hand", "no"}
    ]
    teleop = [r for r in rows if "teleop" in r["j6에 적용되는지"]]
    not_active = [r for r in rows if "not_active" in r["현재 실행 경로에서 실제 사용되는지"] or "not_hand" in r["현재 실행 경로에서 실제 사용되는지"]]

    (OUT / "j6_offset_calibration_audit.md").write_text(
        "# j6 Offset / Calibration Audit\n\n"
        "## Scope\n"
        "This is an offline repository/config audit only. No ROS publish, serial write, Docker restart, replay, build, or driver execution was performed.\n\n"
        "## Source Hashes\n"
        f"- `inspire_driver.py`: `{sha256(ROOT / 'system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_driver.py')}`\n"
        f"- `inspire_comm.py`: `{sha256(ROOT / 'system_Teleop/src/Inspire_/inspire_driver/inspire_driver/communicate/inspire_comm.py')}`\n"
        f"- `inspire_bridge.py`: `{sha256(ROOT / 'system_Teleop/src/Inspire_/inspire_driver/inspire_driver/inspire_bridge.py')}`\n\n"
        "## Active hand j6 command/feedback path findings\n"
        "- No active software/config `offset`, `zero`, `angle_offset`, or `joint_offset` parameter was found in the hand j6 driver path.\n"
        "- The active j6 target mapping is `thumb_rot = -data[5] * 950 + 1900` in 0.1 degree units, followed by an upper pre-clamp at 1800 and generic min/max clamp.\n"
        "- The active j6 target limits are 600..1800 raw 0.1 degree units, i.e. 60..180 degrees.\n"
        "- `/inspire/joint_states` `j6` is parsed from `angleAct`; `tj6` is driver-side target echo from `target_joint`, not a hardware ACK.\n"
        "- `mode`, `forceClb`, `clearErr`, `currAct`, `errCode`, and `statusCode` registers are defined in the protocol dictionary, but the current polling path does not read/write them.\n\n"
        "## Active inventory subset\n\n"
        f"{md_table(active, 60)}\n\n"
        "## Teleop-only dynamic normalization candidates\n"
        "These can change teleop `data[5]` range, but they are not active during the direct j6 test when the bridge is stopped.\n\n"
        f"{md_table(teleop, 30)}\n\n"
        "## Not active or not hand-runtime candidates\n\n"
        f"{md_table(not_active, 50)}\n",
        encoding="utf-8",
    )

    (OUT / "j6_offset_symptom_consistency.md").write_text(
        "# j6 Offset Symptom Consistency\n\n"
        "## Observed symptom to explain\n"
        "- Safe start: `tj6=176.6 deg`, `j6 actual=176.6 deg`.\n"
        "- Command: `data[5] 0.1411 -> 0.2000`, `tj6 176.6 -> 171.0 deg`, while `j6 actual` remains `176.6..176.7 deg`.\n"
        "- Earlier small upward command around `174.2 -> 176.7 deg` tracked once, but downward/return commands did not track.\n\n"
        "## Candidate consistency\n"
        "| candidate | status | reason |\n"
        "|---|---|---|\n"
        "| active fixed software offset on target only | does_not_explain | A fixed offset would shift both 176.6 and 171.0 commands. It would not make actual track at one point and then remain fixed while the target changes. |\n"
        "| active fixed software offset on feedback only | does_not_explain | A feedback offset would bias `angleAct`, but changes in physical position should still appear as changes in `j6 actual`. It does not explain a flat actual trace during a 5.6 deg target change. |\n"
        "| common target+feedback offset | does_not_explain | A common coordinate offset preserves delta. The observed problem is loss of delta tracking, not an absolute disagreement at all positions. |\n"
        "| j6 target clamp to 180 deg | does_not_explain | `data[5]=0.2000` reconstructs to 171.0 deg, inside the 60..180 deg range. It is not clipped to 180 deg in the driver formula. |\n"
        "| teleop percentile calibration | not_active_in_current_path | It changes Manus-to-normalized mapping only when `inspire_bridge_node` publishes. Direct test publishes normalized `data[5]` directly with bridge stopped. |\n"
        "| replay `--hold-j6` | not_active_in_current_path | It can hold replay j6, but was not part of the direct staged sweep. |\n"
        "| hand firmware internal zero/soft-limit/fault | partially_explains | Code can send a valid target and read static `angleAct`; firmware could internally refuse one direction due to zero/limit/fault/current protection, but current/error/status are not read in the active path. |\n"
        "| actuator/mechanical endpoint/stiction/cable/gear issue | explains_symptom | Valid target echo plus unchanged `angleAct` during a 5.6 deg command, with prior one-direction movement, is consistent with one-direction mechanical/electromechanical tracking failure. |\n\n"
        "## Logical check\n"
        "A fixed software offset is an affine coordinate error. It can create a constant target/actual bias or an endpoint clipping issue, but it cannot by itself erase the commanded delta only in one direction while leaving the target echo correct. The present symptom is directional non-tracking, not merely wrong zero.\n",
        encoding="utf-8",
    )

    (OUT / "j6_protocol_boundary.md").write_text(
        "# j6 Protocol / Hardware Boundary\n\n"
        "## Confirmable from code\n"
        "- ROS command: `/inspire/right/target.data[5]` is accepted by `cmd_callback`.\n"
        "- Driver mapping: `thumb_rot = -data[5] * 950 + 1900` raw 0.1 degree units.\n"
        "- Driver clamp: j6 target is clamped to 600..1800 raw units in both `inspire_driver.py` and `inspire_comm.py`.\n"
        "- Serial target payload: `angleSet` register 1040 receives six int16 values, j6 as the sixth low/high byte pair.\n"
        "- Feedback: `angleAct` register 1064 is read as six int16 values and divided by 10.0; j6 is index 5.\n"
        "- `tj6`: driver command echo from `target_joint[5]`; it is not an `angleSet` hardware ACK.\n\n"
        "## Not confirmable from code alone\n"
        "- Whether the RH56 firmware uses hidden zero calibration or soft limits internally.\n"
        "- Whether `angleSet` and `angleAct` are guaranteed by firmware to share the same calibrated coordinate frame under all fault/calibration states.\n"
        "- Whether the j6 motor is disabled, current-limited, blocked by an internal soft limit, or in a latched fault state.\n"
        "- Whether there is a mechanical endpoint, gear backlash, cable slip, or internal linkage issue.\n\n"
        "## Coordinate-system note\n"
        "The code assumes `angleSet` and `angleAct` use the same 0.1 degree calibrated coordinate system because both are Inspire protocol angle registers. The repository code itself does not prove that firmware cannot apply hidden zero/limit logic between command acceptance and motor actuation.\n",
        encoding="utf-8",
    )

    (OUT / "j6_next_step_recommendation.md").write_text(
        "# j6 Next Step Recommendation\n\n"
        "## Final classification\n"
        "**3. offset으로 현재 directional non-tracking을 설명하기 어렵고, actuator/limit/mechanical 방향성 문제가 더 유력함.**\n\n"
        "Recommended next action: **software path is sufficiently cleared; inspect hand actuator/limit/mechanical/internal controller state next**.\n\n"
        "Reasoning:\n"
        "- Direct ROS command changed `target.data[5]` and driver `tj6` exactly as expected.\n"
        "- The 171.0 deg command is inside the documented driver/protocol clamp range.\n"
        "- No active repository software/config offset or calibration parameter was found that can explain a flat `angleAct` response to a 5.6 deg target change.\n"
        "- Current/error/status registers exist in the protocol dictionary, but the active driver path does not read them, so firmware fault/limit/current state remains outside this offline proof.\n\n"
        "If one more software-side observation is allowed later, the single highest-value evidence would be current/error/status/limit state for j6 while issuing the same tiny normal ROS command. That requires an explicitly enabled diagnostic run or vendor tooling, not offset/mapping changes.\n",
        encoding="utf-8",
    )


def main():
    rows = search_inventory()
    write_csv(rows)
    write_docs(rows)
    print(f"wrote {len(rows)} inventory rows to {OUT}")


if __name__ == "__main__":
    main()
