import os
import struct

# C++ struct StateRecordBin layout
FMT = "<8sqQQ30f20f20f6f6f6f6f"

RECORD_SIZE = struct.calcsize(FMT)

pre_x = [0] * 20
cnt = 0
def read_state_record_bin(file_path: str):
    records = []

    file_size = os.path.getsize(file_path)

    if file_size % RECORD_SIZE != 0:
        raise ValueError(
            f"file size ({file_size}) not multiple of record size ({RECORD_SIZE})"
        )

    with open(file_path, "rb") as f:

        while True:

            chunk = f.read(RECORD_SIZE)

            if not chunk:
                break

            values = struct.unpack(FMT, chunk)

            side_raw = values[0]
            side = side_raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")

            t_ns = values[1]
            frame_index = values[2]
            seq = values[3]

            gripper_sensor = list(values[4:34])
            gripper_joint = list(values[34:54])
            target_gripper_joint = list(values[54:74])
            robot_joint = list(values[74:80])
            target_robot_joint = list(values[80:86])
            robot_pose = list(values[86:92])
            robot_ft = list(values[92:98])

            records.append(
                {
                    "side": side,
                    "t_ns": t_ns,
                    "frame_index": frame_index,
                    "seq": seq,
                    "gripper_sensor": gripper_sensor,
                    "gripper_joint": gripper_joint,
                    "target_gripper_joint": target_gripper_joint,
                    "robot_joint": robot_joint,
                    "target_robot_joint": target_robot_joint,
                    "robot_pose": robot_pose,
                    "robot_ft" : robot_ft,
                }
            )

    return records

def print_records(records):

    print(f"record_count = {len(records)}")
    print(f"record_size  = {RECORD_SIZE}")
    print("-" * 120)
    global pre_x, cnt
    for i, rec in enumerate(records):

        print(f"[record {i}]")
        print(
            f"side={rec['side']}, "
            f"t_ns={rec['t_ns']}, "
            f"frame_index={rec['frame_index']}, "
            f"seq={rec['seq']}, "
        )

        ft = ", ".join(f"{x:.3f}" for x in rec["gripper_sensor"])
        gj = ", ".join(f"{x:.3f}" for x in rec["gripper_joint"])

        sp = False
        for i in range(20):
            if(pre_x[i] != rec["gripper_joint"][i]):
                sp = True
              
        if sp:
            cnt+=1

        pre_x = rec["gripper_joint"]
        
        tgj = ", ".join(f"{x:.3f}" for x in rec["target_gripper_joint"])
        rj = ", ".join(f"{x:.3f}" for x in rec["robot_joint"])
        trj = ", ".join(f"{x:.3f}" for x in rec["target_robot_joint"])
        rp = ", ".join(f"{x:.3f}" for x in rec["robot_pose"])
        rf = ", ".join(f"{x:.3f}" for x in rec["robot_ft"])
        
        print(f"gripper_sensor = [{ft}]")
        print(f"gripper_joint= [{gj}]")
        print(f"target_gripper_joint= [{tgj}]")
        print(f"robot_joint  = [{rj}]")
        print(f"target_robot_joint  = [{trj}]")
        print(f"robot_pose   = [{rp}]")
        print(f"robot_ft   = [{rf}]")

        print("-" * 120)

if __name__ == "__main__":
    file_path = "Record/left/frame_data_150.bin"
    # file_path = "Record/right/frame_data_30.bin"
    records = read_state_record_bin(file_path)
    print_records(records)
    print("count : ", cnt)