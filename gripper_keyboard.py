#!/usr/bin/env python3
from time import sleep
import redis
import sys
import signal
import time
import threading
import termios
import tty
import select
from gripper import RobotiqGripper

# 参数配置
INITIAL_POSITION = 0.32  # 遥操作初始位置(INPUT_MIN)
INPUT_MIN = 0.32  # 遥操作初始位置（完全张开）
INPUT_MAX = 0.95
OUTPUT_MIN = 0    # 对应Robotiq 2F-85完全张开
OUTPUT_MAX = 205  # 对应Robotiq 2F-85闭合位置

# 全局变量
manual_control = False  # 是否进入手动控制模式
lock_position = None  # 当前锁定位置，用于保持夹爪原位
manual_control_lock = threading.Lock()  # 线程锁
exit_flag = False  # 程序退出标志

class NonBlockingInput:
    """上下文管理器实现非阻塞输入"""
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # 设置cbreak模式
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    @staticmethod
    def get_char():
        """非阻塞获取单个字符"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

def signal_handler(sig, frame):
    """信号中断处理"""
    global exit_flag
    print("\n程序已中断, 正在清理资源...")
    exit_flag = True
    sys.exit(0)

def map_value(value, in_min, in_max, out_min, out_max):
    """线性映射函数"""
    clamped_value = max(in_min, min(in_max, value))
    return (clamped_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def input_listener():
    """键盘监听线程函数"""
    global manual_control, lock_position, exit_flag
    print("\n[输入监听] 快捷键说明:")
    print("  hold(h): 进入保持模式")
    print("  teleoperation(t): 进入遥操作模式")
    print("  quit(q): 退出程序")
    print("  Ctrl+C: 强制退出")

    with NonBlockingInput() as nbi:
        while not exit_flag:
            char = NonBlockingInput.get_char()
            if char:
                with manual_control_lock:
                    if char == 'h':
                        manual_control = True             
                        print("\n保持模式")
                    elif char == 't':
                        manual_control = False
                        lock_position = None     
                        print("\n遥操作模式")
                    elif char == 'q':
                        print("\n程序退出")
                        exit_flag = True
            time.sleep(0.05)

if __name__ == "__main__":
    # 初始化夹爪
    gripper = RobotiqGripper()
    gripper.activate_gripper()
    time.sleep(1)
    gripper.open_gripper()

    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化Redis
    r = redis.Redis(host="localhost", port=6379, db=0)
    p = r.pubsub()
    p.subscribe("gripper_channel")

    # 启动输入监听线程
    input_thread = threading.Thread(target=input_listener, daemon=True)
    input_thread.start()

    try:
        while not exit_flag:
            # 非阻塞读取Redis消息
            message = p.get_message()
            if message and message["type"] == "message":
                try:
                    current_state = float(message["data"].decode())

                    with manual_control_lock:  # 线程锁
                        if manual_control: # # 保持模式                           
                            if lock_position is None:
                                lock_position = gripper.get_gripper_status()["position"]                  
                                print(f"\n锁定位置: {lock_position}")
                            gripper.move(position=lock_position, speed=0, force=20)
                        else:
                            # 正常控制模式
                            mapped_value = int(map_value(
                                current_state, INPUT_MIN, INPUT_MAX, OUTPUT_MIN, OUTPUT_MAX
                            ))
                            gripper.move(position=mapped_value, speed=255, force=20)
                            lock_position = mapped_value

                except ValueError:
                    print("[错误] 无效的夹爪状态数据")
                except Exception as e:
                    print(f"[异常] 夹爪控制错误: {str(e)}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        exit_flag = True
        print("\n[系统] 正在退出...")
        gripper.open_gripper()
        sys.exit(0)