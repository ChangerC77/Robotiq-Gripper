#!/usr/bin/env python3
import redis
import sys
import signal
import time
import rospy
from std_msgs.msg import Float32
from gripper import RobotiqGripper

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
        self.last_control_state = None  # 记录上一次的控制状态
        
        # 初始化夹爪
        try:
            self.gripper = RobotiqGripper()
            if self.gripper.activate_gripper():
                time.sleep(1)
                if self.gripper.open_gripper():
                    self.gripper_initialized = True
                    rospy.loginfo("夹爪初始化成功")
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

    def signal_handler(self, sig, frame):
        """信号处理函数"""
        self.exit_flag = True
        rospy.loginfo("\n正在退出...")
        if self.gripper_initialized:
            self.gripper.open_gripper()
        sys.exit(0)

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
        
        # 状态变化时才处理
        if new_force_hold != self.force_hold:
            if new_force_hold:
                # 首次超过阈值
                position = self.get_gripper_position()
                if position is not None:
                    self.lock_position = position
                    rospy.loginfo(f"\n力超过阈值({FORCE_THRESHOLD}N): {self.current_force:.2f}N, 保持位置: {self.lock_position}")
            else:
                # 低于阈值
                rospy.loginfo("\n力低于阈值，恢复遥操作控制")
                self.lock_position = None
                
            self.force_hold = new_force_hold

    def move_gripper(self, position):
        """安全移动夹爪"""
        if not self.gripper_initialized or position is None:
            return False
            
        try:
            # 确保位置在有效范围内
            position = max(OUTPUT_MIN, min(OUTPUT_MAX, position))
            
            if self.force_hold and self.lock_position is not None:
                self.gripper.move(
                    position=self.lock_position, 
                    speed=0, 
                    force=HOLD_FORCE
                )
            else:
                self.gripper.move(
                    position=position,
                    speed=MOVE_SPEED,
                    force=MOVE_FORCE
                )
            return True
        except Exception as e:
            rospy.logerr(f"夹爪移动失败: {str(e)}")
            return False

    def run(self):
        """主控制循环"""
        try:
            while not self.exit_flag and not rospy.is_shutdown():
                # 非阻塞读取Redis消息
                message = self.p.get_message()
                if message and message["type"] == "message":
                    try:
                        current_state = float(message["data"].decode())
                        self.last_control_state = current_state  # 记录当前控制状态
                        
                        # 检查是否回到零位
                        if (self.force_hold and 
                            abs(current_state - INPUT_MIN) < ZERO_POSITION_THRESHOLD):
                            rospy.loginfo("\n舵机回到零位，解除保持状态")
                            self.force_hold = False
                            self.lock_position = None
                            # 直接发送张开指令
                            if self.gripper_initialized:
                                self.gripper.move(
                                    position=OUTPUT_MIN,
                                    speed=MOVE_SPEED,
                                    force=MOVE_FORCE
                                )
                            continue
                        
                        # 计算目标位置
                        if INPUT_MAX != INPUT_MIN:  # 避免除零错误
                            mapped_value = int((current_state - INPUT_MIN) * 
                                         (OUTPUT_MAX - OUTPUT_MIN) / 
                                         (INPUT_MAX - INPUT_MIN) + OUTPUT_MIN)
                            mapped_value = max(OUTPUT_MIN, min(OUTPUT_MAX, mapped_value))
                        else:
                            mapped_value = OUTPUT_MIN
                        
                        # 决定使用哪个位置
                        target_pos = self.lock_position if self.force_hold else mapped_value
                        
                        # 移动夹爪
                        if not self.move_gripper(target_pos):
                            rospy.logwarn("夹爪移动未执行")
                            
                    except ValueError:
                        rospy.logwarn("接收到无效的夹爪状态数据")
                    except Exception as e:
                        rospy.logerr(f"控制逻辑错误: {str(e)}")

                time.sleep(0.01)

        except KeyboardInterrupt:
            pass
        finally:
            self.exit_flag = True
            if self.gripper_initialized:
                self.gripper.open_gripper()

if __name__ == "__main__":
    controller = ForceControlledGripper()
    controller.run()