#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove
from std_msgs.msg import Float64MultiArray
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List
import numpy as np

# --- 매핑 유틸리티 ---
RH56_ORDER = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rot")

# ThumbMCPSpread는 벌릴수록(abduction) raw값이 증가 → HIGH=open (손가락 굴곡과 반대 방향)
# 이 목록에 속한 채널은 99th pct = open_ref, 1st pct = close_ref 로 처리한다.
_HIGH_IS_OPEN = frozenset({"thumb_rot"})

WINDOW_SIZE     = 600   # rolling window 크기 (재계산 주기이자 메모리 상한)
MIN_SAMPLES     = 100   # 자동 기준점 활성화에 필요한 최소 샘플 수
LOW_PCT         = 1     # close 기준 percentile
HIGH_PCT        = 99    # open 기준 percentile

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else float(x)

def _type_name(t) -> str:
    s = str(t)
    if "." in s: s = s.split(".")[-1]
    if "/" in s: s = s.split("/")[-1]
    return s

def _erg_dict(manus_msg) -> Dict[str, float]:
    d: Dict[str, float] = {}
    for e in getattr(manus_msg, "ergonomics", []):
        key = _type_name(getattr(e, "type", ""))
        val = float(getattr(e, "value", 0.0))
        d[key] = val
    return d

def _finger_curl_raw(d: Dict[str, float], finger: str) -> float:
    mcp = d.get(f"{finger}MCPStretch", 0.0)
    pip = d.get(f"{finger}PIPStretch", 0.0)
    dip = d.get(f"{finger}DIPStretch", 0.0)
    return 0.4 * mcp + 0.4 * pip + 0.2 * dip

def _thumb_bend_raw(d: Dict[str, float]) -> float:
    # ThumbMCPStretch는 CMC 관절 굴곡으로 ThumbMCPSpread(rotation)와 같은 관절 → 커플링 제거를 위해 제외
    # ThumbPIPStretch = MCP 너클 굴곡, ThumbDIPStretch = IP 끝 관절 굴곡 (순수 굽히기)
    pip = d.get("ThumbPIPStretch", 0.0)
    dip = d.get("ThumbDIPStretch", 0.0)
    return 0.6 * pip + 0.4 * dip

def _thumb_rot_raw(d: Dict[str, float]) -> float:
    return d.get("ThumbMCPSpread", 0.0)

def manus_raw6_from_ergonomics(manus_msg) -> Dict[str, float]:
    d = _erg_dict(manus_msg)
    return {
        "index"         :       _finger_curl_raw(d, "Index"),
        "middle"        :       _finger_curl_raw(d, "Middle"),
        "ring"          :       _finger_curl_raw(d, "Ring"),
        "pinky"         :       _finger_curl_raw(d, "Pinky"),
        "thumb_bend"    :       _thumb_bend_raw(d),
        "thumb_rot"     :       _thumb_rot_raw(d),
    }

# --- Percentile 기반 자동 캘리브레이션 ---
@dataclass
class PercentileReference:
    """
    수집된 raw6 샘플의 LOW_PCT / HIGH_PCT percentile을
    각각 close / open 기준점으로 자동 설정한다.
    """
    buffer: Dict[str, deque] = field(
        default_factory=lambda: {k: deque(maxlen=WINDOW_SIZE) for k in RH56_ORDER}
    )
    open_ref:  Dict[str, float] = field(default_factory=dict)
    close_ref: Dict[str, float] = field(default_factory=dict)
    _push_count: int = 0

    def push(self, raw6: Dict[str, float]) -> None:
        for k in RH56_ORDER:
            self.buffer[k].append(raw6.get(k, 0.0))
        self._push_count += 1
        # MIN_SAMPLES(100개) 이상 쌓이면 100샘플마다 재계산 (~3초 주기, 기존 600샘플=20초보다 빠름)
        if self._push_count >= MIN_SAMPLES and self._push_count % MIN_SAMPLES == 0:
            self._recompute()

    def _recompute(self) -> None:
        if len(self.buffer[RH56_ORDER[0]]) < MIN_SAMPLES:
            return
        for k in RH56_ORDER:
            arr  = np.array(self.buffer[k])
            new_low  = float(np.percentile(arr, LOW_PCT))
            new_high = float(np.percentile(arr, HIGH_PCT))
            # ratchet: 범위가 넓어질 때만 업데이트
            if k in _HIGH_IS_OPEN:
                # thumb_rot (ThumbMCPSpread): 벌릴수록 raw↑ → 99th pct = open, 1st pct = close
                self.open_ref[k]  = max(self.open_ref.get(k,  new_high), new_high)
                self.close_ref[k] = min(self.close_ref.get(k, new_low),  new_low)
            else:
                # 손가락 굴곡: 쥐면 raw↑ → 99th pct = close, 1st pct = open
                self.open_ref[k]  = min(self.open_ref.get(k,  new_low),  new_low)
                self.close_ref[k] = max(self.close_ref.get(k, new_high), new_high)

    @property
    def is_ready(self) -> bool:
        return bool(self.open_ref) and bool(self.close_ref)

    @property
    def sample_count(self) -> int:
        return len(self.buffer[RH56_ORDER[0]])


class DualManusRH56Bridge(Node):
    def __init__(self):
        super().__init__('inspire_bridge_node')

        self.ref_left  = PercentileReference()
        self.ref_right = PercentileReference()
        self.latest_commands: Dict[str, List[float]] = {}
        self.latest_raw6: Dict[str, Dict[str, float]] = {}
        self.latest_erg: Dict[str, Dict[str, float]] = {}   # Manus 원시 ergonomics (ThumbPIPStretch 등)

        # self.create_subscription(ManusGlove, '/manus_glove_0', self.listener_callback, 10)
        self.create_subscription(ManusGlove, '/manus_glove_1', self.listener_callback, 1)

        self.pub_left  = self.create_publisher(Float64MultiArray, '/inspire/left/target',  1)
        self.pub_right = self.create_publisher(Float64MultiArray, '/inspire/right/target', 1)

        # self.create_timer(0.05, self.update_dashboard)

    def map_manus_to_6d(self, raw6: Dict[str, float], ref: PercentileReference) -> List[float]:
        out: List[float] = []
        for key in RH56_ORDER:
            raw = float(raw6.get(key, 0.0))
            o, c = ref.open_ref.get(key), ref.close_ref.get(key)
            if o is None or c is None:
                out.append(0.5)
                continue
            denom = float(o) - float(c)
            out.append(clamp01((raw - float(c)) / denom) if abs(denom) > 1e-9 else 0.0)
        return out

    def listener_callback(self, msg):
        side_str    = "Left" if "Left" in msg.side else "Right"
        current_ref = self.ref_left if side_str == "Left" else self.ref_right

        erg = _erg_dict(msg)
        raw6 = manus_raw6_from_ergonomics(msg)
        current_ref.push(raw6)

        vec01   = self.map_manus_to_6d(raw6, current_ref)
        cmd_msg = Float64MultiArray(data=vec01)
        self.latest_commands[side_str] = vec01
        self.latest_raw6[side_str] = raw6
        self.latest_erg[side_str] = erg

        if side_str == "Left": self.pub_left.publish(cmd_msg)
        else:                  self.pub_right.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = DualManusRH56Bridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
