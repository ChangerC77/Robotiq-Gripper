import numpy as np
import gripper
import time

if __name__ == "__main__":
    # 初始化夹爪
    gripper = gripper.RobotiqGripper()
    
    try:
        # 读取.npy文件
        data = np.load('data/0007.npy')
        positions = data[:, 1].astype(int)  # 确保位置值是整数
        
        print(f"总共读取到 {len(positions)} 个位置点")
        print("开始执行夹爪位置控制...")
        
        # 先移动到初始位置
        gripper.move(position=positions[0], speed=255, force=50)
        time.sleep(1.0)  # 初始移动给足时间
        
        for i in range(1, len(positions)):
            # 计算位置变化量
            delta = abs(positions[i] - positions[i-1])
            
            # 动态调整速度和延迟
            speed = min(255, 100 + delta * 2)  # 变化越大速度越快
            delay = max(0.05, min(1.0, delta / 100.0))  # 基础延迟+变化量调整
            
            # 移动夹爪
            gripper.move(position=positions[i], speed=speed, force=50)
            print(f"移动到位置: {positions[i]}, 速度: {speed}, 延迟: {delay:.2f}s")
            
            time.sleep(delay)
    
    except Exception as e:
        print(f"发生错误: {str(e)}")
    finally:
        # 关闭连接
        gripper.disconnect()
        print("夹爪控制完成，已断开连接")