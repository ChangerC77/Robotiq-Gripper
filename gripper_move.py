import gripper
import time

if __name__ == "__main__":
   gripper = gripper.RobotiqGripper()

   try:
       while True:
          for i in range(0, 205, 1):
               # 移动夹爪到任意位置
               gripper.move(position=i, speed=200, force=20)  # 示例：移动到中间位置
               time.sleep(0.01)

   finally:
        # 关闭连接
        gripper.disconnect()