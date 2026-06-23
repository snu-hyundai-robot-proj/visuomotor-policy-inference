#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# # main.py
import os, sys
import argparse
import time
import threading
import select
import numpy as np

from scenarios import control

from utils.net import NetClient
from utils.parser import NDJSONParser
from utils.dispatcher import Dispatcher
from utils.api import OpenStreamAPI

robot_ip = "192.168.4.152"
robot_port = 49000
major = 1
trial = 0
key_interrupt = ''

GET_JOINTS = "/project/robot/joints/joint_states"
GET_POSITION = "/project/robot/po_cur"
STOP_ROBOT = "session"

def check_key():        ## 리눅스 비동기 키입력
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

def main():
    p = argparse.ArgumentParser(description="Open Stream Client Examples")

    p.add_argument("scenario", choices=["handshake", "monitor", "control", "stop"])
    p.add_argument("--host", default="192.168.1.10")
    p.add_argument("--port", type=int, default=49000)

    net = NetClient("192.168.4.152",49000)
    parser = NDJSONParser()
    dispatcher = Dispatcher()
    api = OpenStreamAPI(net)

    handshake_ok = threading.Event()

    def _on_handshake_ack(m: dict) -> None:
        ok = bool(m.get('ok'))
        print(f"[ack] handshake_ack ok={ok} version={m.get('version')}")
        if ok:
            handshake_ok.set()

    def _on_recv_pose(m: dict) -> None:
        res = m.get('result')
        global trial
        if res.get('_type') == 'Pose':
            rx = res.get('rx')
            ry = res.get('ry')
            rz = res.get('rz')
            x = res.get('x')
            y = res.get('y')
            z = res.get('z')
            trial += 1

    dispatcher.on_type['handshake_ack'] = _on_handshake_ack

    dispatcher.on_type['monitor_ack'] = lambda m: print(
        f"[ack] monitor_ack ok={m.get('ok')} url={m.get('url')} period_ms={m.get('period_ms')}"
    )
    dispatcher.on_type['data'] = _on_recv_pose

    # dispatcher.on_type['data']

    dispatcher.on_error = lambda e: print(
        f"[ERR] code={e.get('error')} message={e.get('message')} hint={e.get('hint')}"
    )

    net.connect()
    net.start_recv_loop(lambda b: parser.feed(b, dispatcher.dispatch))

    start = time.time()
    
    api.handshake(major=major)

    def print_hz():
        global trial
        while True:
            print(f" communication Hz : {trial}")
            trial = 0
            time.sleep(1)

            if key_interrupt == 'q':
                break
    
    def keyboard_loop():
        global key_interrupt
        while True:
            key_interrupt = input()
            if key_interrupt == 'q':
                break

    get_hz = threading.Thread(target=print_hz)
    get_hz.start()
    
    thread_key = threading.Thread(target=keyboard_loop, daemon=True)
    thread_key.start()

    if handshake_ok.wait(timeout=2.0):
        api.monitor(url=GET_POSITION, period_ms=1, args={})

        base_deg = control.http_get_joint_states(robot_ip, http_port=8888, url_type=GET_JOINTS,timeout_sec=1.0)
        print(f"[INFO] base pose joints={len(base_deg)} deg-range={min(base_deg):.2f}..{max(base_deg):.2f}")

        api.get_joint_state()
        # # 4) trajectory 생성 (deg)
        # points_deg = control.generate_sine_trajectory(
        #     base_deg=base_deg,
        #     cycle_sec=3,
        #     amplitude_deg=3,
        #     dt_sec=0.05,
        #     total_sec=3.0,
        #     active_joint_count=6,
        # )
        
        # dt_sec = 0.02
        # poseStep = int(1/dt_sec)

        # points = base_deg

        # print(base_deg)
        # print(len(points))
        # print(points)
        # # time.sleep(3)
        # api.joint_traject_init()

        # # def send_worker():
        # #     while True:
        # #         # api.joint_traject_init()
        # #         # points[5] -= 0.01
        # #         body = {
        # #             "interval": float(dt_sec),
        # #             "time_from_start": float(0),   # 유효한 time_from_start 사용
        # #             "look_ahead_time": float(0.5),
        # #             "point": [float(x) for x in points], # point는 deg (서버가 deg를 rad로 변환)
        # #         }
        # #         print(points)
        # #         api.joint_traject_insert_point(body)
        # #         time.sleep(dt_sec)

        # #         if key_interrupt == 'q':
        # #             break

        # # sender = threading.Thread(target=send_worker)
        # # sender.start()

        # while True:
        #     time.sleep(1)
        #     if key_interrupt == 'q':
        #         break
    else:
        print("FAILED")

    print("CLOSED")

    api.stop(target=STOP_ROBOT)
    net.close()
    get_hz.join()
    thread_key.join()

    # if args.scenario == "handshake":
    #     sc_handshake.run(args.host, args.port, major=args.major)

    # elif args.scenario == "monitor":
    #     sc_monitor.run(
    #         args.host,
    #         args.port,
    #         major=args.major,
    #         url=args.url,
    #         period_ms=args.period_ms,
    #     )

    # elif args.scenario == "control":
    #     sc_control.run(
    #         args.host,
    #         args.port,
    #         major=args.major,
    #         http_port=args.http_port,
    #         cycle_sec=args.cycle_sec,
    #         amplitude_deg=args.amplitude_deg,
    #         dt_sec=args.dt_sec,
    #         total_sec=args.total_duration_sec,
    #         active_joint_count=args.active_joint_count,
    #         look_ahead_time=args.look_ahead_time,
    #     )

    # elif args.scenario == "stop":
    #     sc_stop.run(args.host, args.port, target="session")


if __name__ == "__main__":
    main()
