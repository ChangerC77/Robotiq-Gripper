#!/usr/bin/env python3
from time import sleep
import redis
import sys
import signal
import time
from gripper import RobotiqGripper
import rospy
from std_msgs.msg import Float32

# 参数配置
INITIAL_POSITION = 0.32
INPUT_MIN = 0.32
INPUT_MAX = 0.95
OUTPUT_MIN = 0
OUTPUT_MAX = 205
FORCE_THRESHOLD = 0.5
POSITION_TOLERANCE = 0.02

# 全局变量
manual_control = False
lock_position = None
exit_flag = False
position_pub = None  

def signal_handler(sig, frame):
    """完全保持原有的信号处理函数"""
    global exit_flag
    print("\n程序已中断, 正在清理资源...")
    exit_flag = True
    sys.exit(0)

class TactileSubscriber:
    def __init__(self):
        rospy.init_node('gripper_force_control', anonymous=True)  
        rospy.Subscriber('tactile_0/force', Float32, self.tactile_0_callback)
    
    def tactile_0_callback(self, msg):
        """仅添加绝对值处理，其他保持不变"""
        global manual_control, lock_position
        force = abs(msg.data)
        print("force", force)
     
        if force > FORCE_THRESHOLD:
            manual_control = True
        else:
            manual_control = False
            lock_position = None

def map_value(value, in_min, in_max, out_min, out_max):
    """完全保持原有的映射函数"""
    clamped_value = max(in_min, min(in_max, value))
    return (clamped_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":
    # 初始化
    gripper = RobotiqGripper()
    time.sleep(1)
    gripper.open_gripper()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Redis初始化（
    r = redis.Redis(host="localhost", port=6379, db=0)
    p = r.pubsub()
    p.subscribe("gripper_channel")
    
    # 启动触觉订阅
    subscriber = TactileSubscriber()
    
    # 新增：创建位置发布器
    position_pub = rospy.Publisher('/gripper_position', Float32, queue_size=10)
    
    try:
        while not exit_flag:
            # Redis消息处理
            message = p.get_message()
            if message and message["type"] == "message":
                try:
                    current_state = float(message["data"].decode())
                    timestamp = time.time()  # 新增：获取当前时间戳
                    
                    # 回零检测逻辑
                    if manual_control and lock_position is not None:
                        if abs(current_state - INITIAL_POSITION) < POSITION_TOLERANCE:
                            print("\n[系统] 检测到回零位置，恢复遥操作模式")
                            gripper.open_gripper()
                            manual_control = False
                            lock_position = None
                            continue
                    
                    if manual_control:
                        if lock_position is None:
                            lock_position = gripper.get_gripper_status()["position"]
                            print(f"\n[保持模式] 锁定位置: {lock_position}")
                        gripper.move(position=lock_position, speed=0, force=20)
                        current_pos = lock_position  # 当前夹爪位置
                    else:
                        mapped_value = int(map_value(
                            current_state, INPUT_MIN, INPUT_MAX, OUTPUT_MIN, OUTPUT_MAX
                        ))
                        gripper.move(position=mapped_value, speed=255, force=20)
                        current_pos = mapped_value  # 当前夹爪位置
                    
                    # 新增：发布带时间戳的位置信息
                    position_msg = Float32()
                    position_msg.data = current_pos
                    position_pub.publish(position_msg)
                    
                    # 新增：打印调试信息（可选）
                    print(f"[{timestamp:.3f}] 位置: {current_pos}")
                            
                except Exception as e:
                    print(f"[异常] {str(e)}")
            
            time.sleep(0.01)
            
    finally:
        gripper.open_gripper()