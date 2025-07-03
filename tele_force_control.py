from dexhand_client import DexHandClient
from time import sleep
import redis
import sys
import signal
import time
import argparse
import numpy as np
import os

# 全局控制变量
grasped = False          # 是否已达到目标夹持力
lock_position = 0        # 达到目标力时的锁定位置
INITIAL_POSITION = 0.32  # 遥操作初始位置(INPUT_MIN)

def report_hand_info(client: DexHandClient):
    global grasped, lock_position
    info = client.hand_info
    if info._frame_cnt % 10 == 0:
        print(
            f"Error:{info.error_flag}, nowforce: {info.avg_force:.3f}N nowpos: {info.now_pos:.3f}mm",
            end=" ",
        )
        if info.now_task in ["GOTO", "POSSERVO"]:
            print(f"goalpos : {info.goal_pos:.2f}")
        elif info.now_task in ["FORCESERVO"]:
            print(f"goalforce : {info.goal_force:.2f}")
        else:
            print()
    
    # 检查是否达到目标力（仅在未锁定状态时检测）
    if not grasped and info.avg_force >= TARGET_FORCE:
        print(f"\n达到目标力{TARGET_FORCE}N，进入锁定状态")
        grasped = True
        lock_position = info.now_pos  # 记录当前夹爪位置

def signal_handler(sig, frame):
    print("\n程序已中断，正在清理资源...")
    client.clear_hand_error()
    sleep(0.5)
    client.set_home()
    client.release_hand()
    sys.exit(0)

# 参数配置
INPUT_MIN = 0.32  # 遥操作初始位置（完全张开）
INPUT_MAX = 0.95
OUTPUT_MIN = 0    # 对应DexHand完全张开
OUTPUT_MAX = 47   # 对应DexHand完全闭合
TARGET_FORCE = 12  # 目标夹持力(N)
POSITION_TOLERANCE = 0.02  # 位置判断容差

def map_value(value, in_min, in_max, out_min, out_max):
    """线性映射函数"""
    clamped_value = max(in_min, min(in_max, value))
    return (clamped_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_dir", type=str, default="/home/robotics/workspace/xx_/real/scripts/umi/data_save")
    parser.add_argument("--traj_number", type=int, default=1)
    args = parser.parse_args()

    save_path = os.path.join(args.root_dir, str(args.traj_number).zfill(4), 'gripper')
    os.makedirs(save_path, exist_ok=True)
    
    client = DexHandClient(
        ip="192.168.2.100",
        port=60031,
        recvCallback_hand=report_hand_info,
    )
    client.start_server()
    client.acquire_hand()
    client.set_home()
    client.pos_servo(goal_pos=0, max_f=12)  # 初始化为完全张开
    sleep(1)
    
    signal.signal(signal.SIGINT, signal_handler)
    r = redis.Redis(host='localhost', port=6379, db=0)
    p = r.pubsub()
    p.subscribe('gripper_channel')
    
    while True:
        for message in p.listen():
            if message['type'] == 'message':
                try:
                    current_state = float(message['data'].decode())
                    
                    if grasped:
                        # 锁定状态：检测是否回到初始位置
                        if abs(current_state - INITIAL_POSITION) < POSITION_TOLERANCE:
                            print("\n检测到遥操作回到初始位置，释放夹爪...")
                            grasped = False
                            client.pos_servo(goal_pos=0, max_f=12)  # 完全张开
                            sleep(0.1)
                        else:
                            # 保持锁定位置
                            client.pos_servo(goal_pos=lock_position, max_f=12)
                    else:
                        # 正常遥操作模式
                        mapped_value = map_value(
                            current_state, INPUT_MIN, INPUT_MAX, OUTPUT_MIN, OUTPUT_MAX
                        )
                        client.pos_servo(goal_pos=mapped_value, max_f=12)
                    
                    timestamp = str(int(round(time.time(),3) * 1000))
                    pos = client.hand_info.now_pos
                    save_name = os.path.join(save_path, timestamp + '.npy')
                    np.save(save_name, np.array(pos))
                except ValueError:
                    print("收到无效的夹爪状态数据")