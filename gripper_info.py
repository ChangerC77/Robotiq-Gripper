import gripper 
if __name__ == "__main__":
    gripper = gripper.RobotiqGripper()

    try:
        while True:
            # 调用 get_gripper_status 函数获取状态信息
            status = gripper.get_gripper_status()
            gripper_status = gripper.get_gripper_extended_status()
            if status is not None:
                # 提取所需的信息
                gOBJ = status['gripper_status']['gOBJ']
                current_mA = status['current'] * 10  # 转换为 mA
                position = status['position']

                # 打印结果
                gOBJ_desc = gripper_status['gripper_status']['gOBJ_desc']
                print(f"gOBJ_desc 的值是: {gOBJ_desc}")
                print(f"物体检测状态 (gOBJ): {gOBJ}")
                print(f"当前电流 (mA): {current_mA} mA")
                print(f"当前夹爪位置: {position}")
            else:
                print("无法获取夹爪状态信息！")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 断开连接
        gripper.disconnect()