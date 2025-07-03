#!/usr/bin/env python3
import redis
import sys
import signal
import time
import rospy
from std_msgs.msg import Float32
from gripper import RobotiqGripper
import argparse
import os
import numpy as np
import select
import termios
import tty

# 参数配置
INPUT_MIN = 0.32    # 遥操作最小输入值(完全张开)
INPUT_MAX = 0.95    # 遥操作最大输入值
OUTPUT_MIN = 0      # 对应夹爪完全张开
OUTPUT_MAX = 205    # 对应夹爪完全闭合
FORCE_THRESHOLD = 5.0  # 触发保持的力阈值(N)
HOLD_FORCE = 20     # 保持时的夹持力
MOVE_FORCE = 20     # 移动时的夹持力
MOVE_SPEED = 255    # 移动速度
ZERO_POSITION_THRESHOLD = 0.1  # 定义舵机零位的阈值范围
RECORD_INTERVAL = 0.01  # 记录间隔时间(秒)

class ForceControlledGripper:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('auto_force_gripper_control')
        
        # 初始化变量
        self.exit_flag = False
        self.force_hold = False
        self.lock_position = None
        self.current_force = 0.0
        self.gripper_initialized = False
        self.last_control_state = None
        self.last_position = None
        self.position_history = []
        self.save_path = None
        self.start_time = time.time()  # 记录程序启动时间
        self.last_record_time = 0.0    # 上次记录时间
        
        # 初始化键盘输入
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # 初始化夹爪
        try:
            self.gripper = RobotiqGripper()
            if self.gripper.activate_gripper():
                time.sleep(1)
                if self.gripper.open_gripper():
                    self.gripper_initialized = True
                    rospy.loginfo("夹爪初始化成功")
                    # 记录初始位置
                    init_pos = self.get_gripper_position()
                    if init_pos is not None:
                        self.position_history.append((0.0, init_pos))  # 初始时间设为0
                        self.last_position = init_pos
                        self.last_record_time = 0.0
                else:
                    rospy.logerr("夹爪打开失败")
            else:
                rospy.logerr("夹爪激活失败")
        except Exception as e:
            rospy.logerr(f"夹爪初始化错误: {str(e)}")
        
        # 订阅力话题
        rospy.Subscriber('tactile_force', Float32, self.force_callback)
        
        # 初始化Redis
        self.r = redis.Redis(host="localhost", port=6379, db=0)
        self.p = self.r.pubsub()
        self.p.subscribe("gripper_channel")
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        rospy.loginfo("力反馈夹爪控制器已启动")
        rospy.loginfo("按 'q' 键退出并保存数据")

    def signal_handler(self, sig, frame):
        """信号处理函数"""
        self.exit_flag = True
        rospy.loginfo("\n正在退出...")
        if self.gripper_initialized:
            self.gripper.open_gripper()
        self.save_position_history()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        sys.exit(0)

    def get_key(self):
        """非阻塞获取键盘输入"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def get_next_filename(self):
        """获取下一个可用的文件名"""
        existing_files = [f for f in os.listdir(self.save_path) 
                        if f.endswith('.npy') and f[:-4].isdigit()]
        if not existing_files:
            return "0001.npy"
        
        max_num = max(int(f[:-4]) for f in existing_files)
        return f"{max_num + 1:04d}.npy"

    def get_gripper_position(self):
        """安全获取夹爪位置"""
        if not self.gripper_initialized:
            return None
            
        try:
            status = self.gripper.get_gripper_status()
            if status and isinstance(status, dict) and 'position' in status:
                return status['position']
            rospy.logwarn("获取夹爪位置失败: 状态无效")
            return None
        except Exception as e:
            rospy.logwarn(f"获取夹爪位置异常: {str(e)}")
            return None

    def force_callback(self, msg):
        """力数据回调函数"""
        if not self.gripper_initialized:
            return
            
        self.current_force = msg.data
        new_force_hold = self.current_force > FORCE_THRESHOLD
        
        if new_force_hold != self.force_hold:
            if new_force_hold:
                position = self.get_gripper_position()
                if position is not None:
                    self.lock_position = position
                    rospy.loginfo(f"\n力超过阈值({FORCE_THRESHOLD}N): {self.current_force:.2f}N, 保持位置: {self.lock_position}")
            else:
                rospy.loginfo("\n力低于阈值，恢复遥操作控制")
                self.lock_position = None
                
            self.force_hold = new_force_hold

    def move_gripper(self, position):
        """安全移动夹爪"""
        if not self.gripper_initialized or position is None:
            return False
            
        try:
            position = max(OUTPUT_MIN, min(OUTPUT_MAX, position))
            
            current_time = time.time() - self.start_time
            if current_time - self.last_record_time >= RECORD_INTERVAL:
                self.position_history.append((current_time, position))
                self.last_record_time = current_time
            
            if position != self.last_position:
                self.last_position = position
            
            if self.force_hold and self.lock_position is not None:
                self.gripper.move(position=self.lock_position, speed=0, force=HOLD_FORCE)
            else:
                self.gripper.move(position=position, speed=MOVE_SPEED, force=MOVE_FORCE)
            return True
        except Exception as e:
            rospy.logerr(f"夹爪移动失败: {str(e)}")
            return False

    def save_position_history(self):
        """保存位置历史记录到文件"""
        if not self.position_history:
            rospy.logwarn("没有位置历史数据需要保存")
            return
            
        try:
            # 确保时间戳连续且单调递增
            data = np.array(self.position_history, dtype=np.float32)
            
            # 对数据进行插值处理，确保均匀时间间隔
            if len(data) > 1:
                start_time = data[0, 0]
                end_time = data[-1, 0]
                num_points = int((end_time - start_time) / RECORD_INTERVAL) + 1
                
                # 创建均匀时间序列
                uniform_times = np.linspace(start_time, end_time, num_points)
                
                # 线性插值位置数据
                uniform_positions = np.interp(uniform_times, data[:, 0], data[:, 1])
                
                # 组合成新数组
                data = np.column_stack((uniform_times, uniform_positions))
            
            filename = os.path.join(self.save_path, self.get_next_filename())
            
            os.makedirs(self.save_path, exist_ok=True)
            np.save(filename, data)
            rospy.loginfo(f"夹爪位置历史已保存到: {filename}")
            print(f"记录的总帧数: {len(data)}")  # 打印总记录帧数
        except Exception as e:
            rospy.logerr(f"保存位置历史失败: {str(e)}")

    def run(self):
        """主控制循环"""
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while not self.exit_flag and not rospy.is_shutdown():
                key = self.get_key()
                if key == 'q':
                    rospy.loginfo("用户请求退出...")
                    self.exit_flag = True
                    break
                
                message = self.p.get_message()
                if message and message["type"] == "message":
                    try:
                        current_state = float(message["data"].decode())
                        self.last_control_state = current_state
                        
                        if (self.force_hold and 
                            abs(current_state - INPUT_MIN) < ZERO_POSITION_THRESHOLD):
                            rospy.loginfo("\n舵机回到零位，解除保持状态")
                            self.force_hold = False
                            self.lock_position = None
                            if self.gripper_initialized:
                                self.gripper.move(
                                    position=OUTPUT_MIN,
                                    speed=MOVE_SPEED,
                                    force=MOVE_FORCE
                                )
                            continue
                        
                        if INPUT_MAX != INPUT_MIN:
                            mapped_value = int((current_state - INPUT_MIN) * 
                                         (OUTPUT_MAX - OUTPUT_MIN) / 
                                         (INPUT_MAX - INPUT_MIN) + OUTPUT_MIN)
                            mapped_value = max(OUTPUT_MIN, min(OUTPUT_MAX, mapped_value))
                        else:
                            mapped_value = OUTPUT_MIN
                        
                        target_pos = self.lock_position if self.force_hold else mapped_value
                        
                        if not self.move_gripper(target_pos):
                            rospy.logwarn("夹爪移动未执行")
                            
                    except ValueError:
                        rospy.logwarn("接收到无效的夹爪状态数据")
                    except Exception as e:
                        rospy.logerr(f"控制逻辑错误: {str(e)}")

                time.sleep(0.001)  # 更小的循环延迟

        except KeyboardInterrupt:
            pass
        finally:
            self.exit_flag = True
            if self.gripper_initialized:
                self.gripper.open_gripper()
            self.save_position_history()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_dir", type=str, default="data")
    parser.add_argument("--record_interval", type=float, default=0.01, 
                       help="记录间隔时间(秒)")
    args = parser.parse_args()

    # 处理路径
    expanded_root = os.path.expanduser(args.root_dir)
    os.makedirs(expanded_root, exist_ok=True)
    
    RECORD_INTERVAL = args.record_interval  # 更新全局记录间隔
    
    controller = ForceControlledGripper()
    controller.save_path = expanded_root  # 直接使用data目录
    controller.run()