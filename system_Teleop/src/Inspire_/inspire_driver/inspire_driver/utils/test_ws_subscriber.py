#!/usr/bin/env python3
"""
rosbridge WebSocket 구독 테스트.
컨테이너가 실행 중인 상태에서 호스트에서 실행하세요:

    pip install roslibpy
    python3 test_ws_subscriber.py
"""

import time
import roslibpy

RH56_ORDER = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rot")

latest = {"Left": None, "Right": None}


def make_callback(side):
    def callback(msg):
        latest[side] = msg["data"]
    return callback


def print_data():
    print("\n" + "=" * 60)
    print(f"  {time.strftime('%H:%M:%S')}")
    for side in ["Left", "Right"]:
        data = latest[side]
        if data is None:
            print(f"  [{side}] 데이터 없음 (아직 수신 안됨)")
        else:
            vals = " | ".join(f"{name}: {v:.2f}" for name, v in zip(RH56_ORDER, data))
            print(f"  [{side}] {vals}")
    print("=" * 60)


def main():
    client = roslibpy.Ros(host="localhost", port=9091)
    client.run()

    print("rosbridge 연결됨. 5초마다 출력합니다. (Ctrl+C로 종료)")

    roslibpy.Topic(
        client, "/inspire_hand/left/target", "std_msgs/Float64MultiArray"
    ).subscribe(make_callback("Left"))

    roslibpy.Topic(
        client, "/inspire_hand/right/target", "std_msgs/Float64MultiArray"
    ).subscribe(make_callback("Right"))

    try:
        while client.is_connected:
            print_data()
            time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        client.terminate()
        print("종료.")


# if __name__ == "__main__":
#     main()
