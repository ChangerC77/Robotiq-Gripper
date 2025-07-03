import redis
import sys
import signal
import time
import threading
from gripper import RobotiqGripper

# 动态添加路径
import os
import sys
collect_path = os.path.expanduser("~/workspace/xx_/real/scripts/umi")
sys.path.append(collect_path)
from tac_sensor import Tacsensors

class ForceControlledGripper:
    def __init__(self):
        # 初始化触觉传感器
        self.tacsensors = Tacsensors(SNs=['DL1-GWM0001', 'DL1-GWM0002'], port=9988)
        
        # 力阈值参数 (单位: N)
        self.FORCE_THRESHOLD = 1000.0   # 触发力保护的阈值
        
        # 控制参数
        self.INPUT_MIN = 0.32    # 遥操作输入最小值(完全张开)
        self.INPUT_MAX = 0.95    # 遥操作输入最大值(完全闭合)
        self.OUTPUT_MIN = 0      # 夹爪物理位置最小值(完全张开)
        self.OUTPUT_MAX = 205    # 夹爪物理位置最大值(完全闭合)
        
        # 状态变量
        self.current_force = [0.0, 0.0]  # [左, 右]传感器力值
        self.force_exceeded = False
        self.lock = threading.Lock()
        self.last_safe_position = self.OUTPUT_MIN  # 记录最后安全位置
        
        # 初始化夹爪
        self.gripper = RobotiqGripper()
        self.gripper.activate_gripper()
        time.sleep(1)
        self.gripper.open_gripper()
        
        # Redis通信设置
        self.redis_conn = redis.Redis(host='localhost', port=6379, db=0)
        self.redis_pubsub = self.redis_conn.pubsub()
        self.redis_pubsub.subscribe('gripper_channel')
        
        # 信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        print("系统初始化完成")

    def signal_handler(self, sig, frame):
        """安全退出处理"""
        print("\n正在安全停止...")
        self.gripper.open_gripper()
        sys.exit(0)

    def map_value(self, value):
        """线性映射函数"""
        return int((max(self.INPUT_MIN, min(self.INPUT_MAX, value)) - self.INPUT_MIN) * \
               (self.OUTPUT_MAX - self.OUTPUT_MIN) / (self.INPUT_MAX - self.INPUT_MIN) + self.OUTPUT_MIN)

    def update_force_status(self, left_force, right_force):
        """更新力传感器状态并检查阈值"""
        with self.lock:
            self.current_force = [left_force, right_force]
            # 实时输出力信息
            print(f"实时力反馈 - 左: {left_force:.2f}N, 右: {right_force:.2f}N", end='\r')
            
            # 使用绝对值进行阈值检查
            abs_left = abs(left_force)
            abs_right = abs(right_force)
            force_exceeded = (abs_left > self.FORCE_THRESHOLD or 
                             abs_right > self.FORCE_THRESHOLD)
            
            # 状态变化时打印信息
            if force_exceeded and not self.force_exceeded:
                print(f"\n! 力超过阈值: |左|={abs_left:.1f}N, |右|={abs_right:.1f}N")
                self.last_safe_position = self.gripper.get_gripper_status()["position"]  # 记录当前位置
            
            self.force_exceeded = force_exceeded

    def force_monitor(self):
        """触觉传感器监测线程"""
        print("力监测线程启动")
        while True:
            try:
                left_force, right_force = self.tacsensors.get_force()
                if left_force is not None and right_force is not None:
                    self.update_force_status(left_force, right_force)
                time.sleep(0.01)  # 10ms更新频率
            except Exception as e:
                print(f"\n力传感器错误: {str(e)}")
                time.sleep(0.1)

    def teleop_control(self):
        """主控制循环"""
        print("遥操作控制线程启动")
        while True:
            # 处理Redis控制命令
            message = self.redis_pubsub.get_message()
            if message and message['type'] == 'message':
                try:
                    cmd_value = float(message['data'].decode())
                    
                    with self.lock:
                        if not self.force_exceeded:
                            target_pos = self.map_value(cmd_value)
                            self.gripper.move(position=target_pos, speed=255, force=20)
                            self.last_safe_position = target_pos  # 更新最后安全位置
                        else:
                            # 力超过阈值时保持最后安全位置
                            self.gripper.move(position=self.last_safe_position, speed=255, force=20)
                            print("操作已锁定: 保持最后安全位置")
                
                except ValueError:
                    print("\n错误: 无效的控制命令")
                except Exception as e:
                    print(f"\n控制错误: {str(e)}")
            
            time.sleep(0.001)  # 1ms控制周期

    def run(self):
        """启动主程序"""
        # 启动力监测线程
        monitor_thread = threading.Thread(
            target=self.force_monitor,
            daemon=True
        )
        monitor_thread.start()
        
        # 主线程运行控制循环
        self.teleop_control()

if __name__ == '__main__':
    controller = ForceControlledGripper()
    print("夹爪力控制系统启动 (Ctrl+C退出)")
    print(f"力阈值: {controller.FORCE_THRESHOLD}N (绝对值比较)")
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        controller.gripper.open_gripper()