# scenarios/control.py
import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple
from urllib.request import urlopen
from urllib.error import URLError, HTTPError

from hdr_stream.utils.net import NetClient
from hdr_stream.utils.parser import NDJSONParser
from hdr_stream.utils.dispatcher import Dispatcher
from hdr_stream.utils.api import OpenStreamAPI
from hdr_stream.utils.motion import generate_sine_trajectory, save_trajectory  # rad_to_deg 제거


def http_get_joint_states(host: str, *, http_port: int = 8888, url_type:str,timeout_sec: float = 1.0) -> List[float]:
    """
    /project/robot/joints/joint_states 를 HTTP GET으로 조회해 joint positions(deg) 리스트를 반환한다.

    (서버 C++ 구현 기준)
    - position: deg 단위로 내려옴
    - velocity: deg/s
    - effort: Nm
    """
    url = f"http://{host}:{http_port}{url_type}"

    try:
        with urlopen(url, timeout=timeout_sec) as r:
            raw = r.read().decode("utf-8")
        data = json.loads(raw)
    except (HTTPError, URLError, TimeoutError) as e:
        raise RuntimeError(f"HTTP GET failed: {url} ({e})") from e
    except json.JSONDecodeError as e:
        raise RuntimeError(f"HTTP response is not valid JSON: {raw[:200]!r}") from e

    q: List[float] = []

    if isinstance(data, list):
        q = [float(v) for v in data if isinstance(v, (int, float))]

    elif isinstance(data, dict):
        # C++ 구현은 {"position":[deg...], "velocity":[deg/s...], "effort":[Nm...]} 형태
        if "position" in data and isinstance(data["position"], list):
            q = [float(v) for v in data["position"] if isinstance(v, (int, float))]
        else:
            # e.g. {"j1": 10.0, "j2": 20.0, ...} 형태도 방어
            items: List[Tuple[int, float]] = []
            for k, v in data.items():
                if not isinstance(v, (int, float)):
                    continue
                if isinstance(k, str) and k.startswith("j"):
                    try:
                        idx = int(k[1:])
                        items.append((idx, float(v)))
                    except ValueError:
                        continue
            q = [v for _, v in sorted(items, key=lambda x: x[0])]

    if not q:
        raise RuntimeError(f"Cannot extract joint positions from response: {data!r}")

    return q


def run(
    host: str,
    port: int,
    *,
    major: int = 1,
    http_port: int = 8888,
    # trajectory
    cycle_sec: float = 1.0,
    amplitude_deg: float = 5.0,
    dt_sec: float = 0.02,
    total_sec: float = 1.0,
    active_joint_count: Optional[int] = 6,
    # control timing
    look_ahead_time: float = 0.1,
) -> None:
    net = NetClient(host, port)
    parser = NDJSONParser()
    dispatcher = Dispatcher()
    api = OpenStreamAPI(net)

    handshake_ok = {"ok": False}

    def on_handshake_ack(m: dict) -> None:
        ok = bool(m.get("ok"))
        handshake_ok["ok"] = ok
        print(f"[ack] handshake_ack ok={ok} version={m.get('version')}")

    dispatcher.on_type["handshake_ack"] = on_handshake_ack
    dispatcher.on_error = lambda e: print(f"[ERR] {e}")

    # 1) connect + recv loop
    net.connect()
    net.start_recv_loop(lambda b: parser.feed(b, dispatcher.dispatch))

    # 2) handshake
    api.handshake(major=major)

    t_wait = time.time() + 2.0
    while time.time() < t_wait and not handshake_ok["ok"]:
        time.sleep(0.01)

    if not handshake_ok["ok"]:
        print("[ERR] handshake_ack not received; aborting.")
        net.close()
        return

    # 3) base pose (deg) via HTTP  <-- 여기 핵심
    base_deg = http_get_joint_states(host, http_port=http_port, timeout_sec=1.0)
    print(f"[INFO] base pose joints={len(base_deg)} deg-range={min(base_deg):.2f}..{max(base_deg):.2f}")

    # 4) trajectory 생성 (deg)
    points_deg = generate_sine_trajectory(
        base_deg=base_deg,
        cycle_sec=cycle_sec,
        amplitude_deg=amplitude_deg,
        dt_sec=dt_sec,
        total_sec=total_sec,
        active_joint_count=active_joint_count,
    )

    saved_path = save_trajectory(points_deg, dt_sec, base_dir="data")
    print(f"[INFO] trajectory saved: {saved_path} (points={len(points_deg)}, dt={dt_sec})")

    # 5) CONTROL init
    api.joint_traject_init()

    # 6) CONTROL insert_point streaming
    t0 = time.time()
    for i, point_deg in enumerate(points_deg):
        body = {
            "interval": float(dt_sec),
            "time_from_start": float(i * dt_sec),   # 유효한 time_from_start 사용
            "look_ahead_time": float(look_ahead_time),
            "point": [float(x) for x in point_deg], # point는 deg (서버가 deg를 rad로 변환)
        }
        api.joint_traject_insert_point(body)

        # dt에 맞춰 송신 (단순 예제)
        target = t0 + (i + 1) * dt_sec
        remain = target - time.time()
        if remain > 0:
            time.sleep(remain)

    net.close()