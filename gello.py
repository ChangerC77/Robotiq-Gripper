from time import sleep
import redis
import sys
import signal
import time
from gripper import RobotiqGripper

class GelloTeleoperation:
    def __init__(self):
        # 参数配置
        self.INITIAL_POSITION = 0.32  # 遥操作初始位置(INPUT_MIN)
        self.INPUT_MIN = 0.32  # 遥操作初始位置（完全张开）
        self.INPUT_MAX = 0.95
        self.OUTPUT_MIN = 0    # 对应Robotiq 2F-85完全张开
        self.OUTPUT_MAX = 205  # 对应Robotiq 2F-85闭合位置
        
        # 初始化夹爪
        self.gripper = RobotiqGripper()
        self.gripper.activate_gripper()
        time.sleep(1)
        self.gripper.open_gripper()
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 初始化Redis连接
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.p = self.r.pubsub()
        self.p.subscribe('gripper_channel')

    def signal_handler(self, sig, frame):
        """处理中断信号"""
        print("\n程序已中断, 正在清理资源...")
        sys.exit(0)

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """线性映射函数"""
        clamped_value = max(in_min, min(in_max, value))
        return (clamped_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
           
    def run(self, speed, force):
        """主运行循环"""
        try:
            while True:
                # 非阻塞式 Redis 消息读取
                message = self.p.get_message()
                if message and message['type'] == 'message':
                    current_state = float(message['data'].decode())
                    # 映射 Gello 输入值到夹爪位置范围
                    mapped_value = int(self.map_value(
                        current_state, self.INPUT_MIN, self.INPUT_MAX, self.OUTPUT_MIN, self.OUTPUT_MAX
                    ))
                    print("mapped_value:", mapped_value)
                    
                    # 控制夹爪实时移动
                    self.gripper.move(position=mapped_value, speed=speed, force=force)
                
                # 短暂延迟，降低 CPU 占用
                time.sleep(0.001)

        except ValueError:
            print("收到无效的夹爪状态数据")
        
if __name__ == "__main__":
    gello = GelloTeleoperation()
    gello.run(speed=255, force=20)