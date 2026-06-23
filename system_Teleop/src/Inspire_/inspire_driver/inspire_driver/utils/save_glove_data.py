#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove
import threading
import time
import os

class DualGloveDashboard(Node):
    def __init__(self):
        super().__init__('dual_glove_dashboard')
        
        # 1. 모든 데이터를 담을 파이썬 리스트
        self.glove_data_list = []
        
        # 2. 양손의 가장 최신 데이터를 저장할 딕셔너리
        self.latest_samples = {} # { 'Left': msg, 'Right': msg }

        # 양손 토픽 구독
        self.create_subscription(ManusGlove, '/manus_glove_0', self.listener_callback, 10)
        self.create_subscription(ManusGlove, '/manus_glove_1', self.listener_callback, 10)

        # 3. 화면을 갱신할 타이머 (0.2초마다 화면 업데이트 = 5Hz)
        self.timer = self.create_timer(2, self.update_dashboard)
        
        print("양손 동기화 대시보드를 준비 중입니다...")

    def listener_callback(self, msg):
        # 모든 데이터는 리스트에 무조건 저장
        sample = {
            'timestamp': time.time(),
            'side': msg.side,
            'glove_id': msg.glove_id,
            'nodes': [
                {
                    'id': n.node_id, 'chain': n.chain_type, 'joint': n.joint_type,
                    'pos': [n.pose.position.x, n.pose.position.y, n.pose.position.z],
                    'rot': [n.pose.orientation.x, n.pose.orientation.y, n.pose.orientation.z, n.pose.orientation.w]
                } for n in msg.raw_nodes
            ]
        }
        self.glove_data_list.append(sample)
        
        # 최신 데이터 업데이트
        self.latest_samples[msg.side] = msg

    def update_dashboard(self):
        """화면을 지우고 양손 데이터를 동시에 출력"""
        if not self.latest_samples:
            return

        # 터미널 화면 청소 (Windows는 'cls')
        os.system('clear')

        curr_time = time.strftime("%H:%M:%S", time.localtime())
        print(f" { '■'*35 } MANUS DUAL GLOVE DASHBOARD { '■'*35 }")
        print(f" 시각: {curr_time} | 총 누적 프레임: {len(self.glove_data_list)}")
        print(f" { '='*100 }")

        # 저장된 side(Left/Right) 순서대로 출력
        for side in sorted(self.latest_samples.keys()):
            msg = self.latest_samples[side]
            print(f"\n [ {side.upper()} HAND - ID: {msg.glove_id} ]")
            print(f"{'-'*110}")
            header = f"{'ID':^4} | {'Chain':^8} | {'Joint':^8} | {'Pos X':^8} | {'Pos Y':^8} | {'Pos Z':^8} | {'Rot X':^7} | {'Rot Y':^7} | {'Rot Z':^7} | {'Rot W':^7}"
            print(header)
            print(f"{'-'*110}")

            # 가독성을 위해 상위 10개 노드만 출력하거나 전체 출력
            # 여기서는 전체 25개 노드 중 핵심 노드들을 보기 좋게 출력합니다.
            for n in msg.raw_nodes:
                p, r = n.pose.position, n.pose.orientation
                row = (f"{n.node_id:^4} | {n.chain_type:^8} | {n.joint_type:^8} | "
                       f"{p.x:8.3f} | {p.y:8.3f} | {p.z:8.3f} | "
                       f"{r.x:7.2f} | {r.y:7.2f} | {r.z:7.2f} | {r.w:7.2f}")
                print(row)
            print(f"{'-'*110}")

# def main(args=None):
#     rclpy.init(args=args)
#     collector = DualGloveDashboard()

#     spin_thread = threading.Thread(target=rclpy.spin, args=(collector,), daemon=True)
#     spin_thread.start()

#     try:
#         while rclpy.ok():
#             time.sleep(0.1)
#     except KeyboardInterrupt:
#         print(f"\n✅ 수집 완료. 총 {len(collector.glove_data_list)} 프레임이 리스트에 저장되었습니다.")
#     finally:
#         collector.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()