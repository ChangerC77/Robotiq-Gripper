from time import sleep
import redis
import sys
import signal
import time
from gripper import RobotiqGripper

# 参数配置
INITIAL_POSITION = 0.32  # 遥操作初始位置(INPUT_MIN)
INPUT_MIN = 0.32  # 遥操作初始位置（完全张开）
INPUT_MAX = 0.95
OUTPUT_MIN = 0    # 对应Robotiq 2F-85完全张开
OUTPUT_MAX = 205   # 对应Robotiq 2F-85闭合位置

def signal_handler(sig, frame):
    print("\n程序已中断, 正在清理资源...")
    sys.exit(0)

def map_value(value, in_min, in_max, out_min, out_max):
    """线性映射函数"""
    clamped_value = max(in_min, min(in_max, value))
    return (clamped_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":
    gripper = RobotiqGripper()
    gripper.open_gripper()

    grasped = False
    signal.signal(signal.SIGINT, signal_handler)
    r = redis.Redis(host='localhost', port=6379, db=0)
    p = r.pubsub()
    p.subscribe('gripper_channel')

    while True:
        # 非阻塞式 Redis 消息读取
        message = p.get_message()
        if message and message['type'] == 'message':
            try:
                current_state = float(message['data'].decode())
                # print("current_state: ", current_state)

                # 映射 Gello 输入值到夹爪位置范围
                mapped_value = int(map_value(
                    current_state, INPUT_MIN, INPUT_MAX, OUTPUT_MIN, OUTPUT_MAX
                ))
                print("mapped_value:", mapped_value)

                # 控制 B 夹爪实时移动
                gripper.move(position=mapped_value, speed=255, force=20)

            except ValueError:
                print("收到无效的夹爪状态数据")

        # 短暂延迟，降低 CPU 占用
        time.sleep(0.001)