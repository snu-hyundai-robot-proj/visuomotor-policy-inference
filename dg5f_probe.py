#!/usr/bin/env python3
"""Standalone DG5F (Tesollo Delto Gripper-5F) connectivity probe — raw TCP, no deps.

Verifies the hand hardware directly (no ROS2 / ros2_control / container needed): connects to
the gripper's TCP server and asks for its version+model. Use it to answer "is the DG5F actually
on the network and responding?" independent of the whole stack.

  ./dg5f_probe.py                 # probe 192.168.4.73:502
  ./dg5f_probe.py --ip 192.168.4.73

NOTE: the DG5F does NOT speak Modbus — it uses Tesollo's own TCP protocol:
  packet = Length(2, big-endian) + CMD(1) + payload
  GET_VERSION (0x08): request [0x00,0x03,0x08] -> response 7 bytes:
      Length(2) + CMD(1) + Model(2) + Version(2)
(from the SDK: delto_tcp_comm/src/delto_developer_TCP.cpp)
"""
import argparse
import socket
import struct
import sys

GET_VERSION_CMD = 0x08
GET_DATA_CMD = 0x01
MODELS = {
    0x3F01: "DG3F-B", 0x3F02: "DG3F-M", 0x4F02: "DG4F",
    0x5F02: "DG5F", 0x5F12: "DG5F-L (left)", 0x5F22: "DG5F-R (right)",
}


def recvn(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed mid-response")
        buf += chunk
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.4.73")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--timeout", type=float, default=3.0)
    args = ap.parse_args()

    print(f"[dg5f_probe] connecting {args.ip}:{args.port} (Tesollo TCP) ...")
    try:
        sock = socket.create_connection((args.ip, args.port), timeout=args.timeout)
        sock.settimeout(args.timeout)
    except Exception as e:
        print(f"  ❌ CONNECT FAILED: {e}")
        print("     -> DG5F not reachable. Check: hand powered + ethernet on the robot switch,")
        print(f"        and the host is NOT holding {args.ip} (that IP belongs to the hand).")
        sys.exit(1)
    print("  ✅ TCP connected.")

    try:
        sock.sendall(bytes([0x00, 0x03, GET_VERSION_CMD]))     # GET_VERSION
        resp = recvn(sock, 7)                                  # Len(2)+CMD(1)+Model(2)+Ver(2)
        cmd = resp[2]
        model = struct.unpack(">H", resp[3:5])[0]
        ver = struct.unpack(">H", resp[5:7])[0]
        if cmd != GET_VERSION_CMD:
            print(f"  ⚠️  unexpected reply CMD=0x{cmd:02X} (raw {resp.hex()})")
            sys.exit(2)
        name = MODELS.get(model, f"unknown(0x{model:04X})")
        print(f"  ✅ DG5F ALIVE — model: {name}  (0x{model:04X})   firmware: {ver >> 8}.{ver & 0xFF}")
        print("  -> hand hardware is up. Bring the stack up:  ./up.sh left")
    except socket.timeout:
        print("  ❌ connected but NO REPLY to GET_VERSION (timeout).")
        print("     TCP is open but the gripper controller isn't answering — power-cycle the hand.")
        sys.exit(1)
    except Exception as e:
        print(f"  ❌ probe failed: {e}")
        sys.exit(1)
    finally:
        sock.close()
    print("[dg5f_probe] done")


if __name__ == "__main__":
    main()
