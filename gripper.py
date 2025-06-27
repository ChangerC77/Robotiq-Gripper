import serial
import time
import binascii

class RobotiqGripper:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        """
        初始化串口连接，设置默认通信参数。
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
    
    def send_command(self, command):
        """
        发送指令到夹爪，并读取返回数据
        """
        self.ser.write(command)
        time.sleep(0.05)
        response = self.ser.read_all()
        return response

    def receive(self, command):
        """
        接收夹爪返回的数据
        """
        self.ser.write(command)
        time.sleep(0.05)
        response = self.ser.read_all()
        return response

    def activate_gripper(self):
        """
        激活夹爪
        """
        command = b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30'
        response = self.send_command(command)
        print(f"Activate Response: {binascii.hexlify(response)}")
        return response

    def deactivate_gripper(self):
        """
        复位夹爪
        """
        command = b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30'
        response = self.send_command(command)
        print(f"Deactivate Response: {binascii.hexlify(response)}")
        return response

    def close_gripper(self):
        """
        关闭夹爪
        """
        command = b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29'
        response = self.send_command(command)
        print(f"Close Gripper Response: {binascii.hexlify(response)}")
        return response

    def open_gripper(self):
        """
        打开夹爪
        """
        command = b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19'
        response = self.send_command(command)
        print(f"Open Gripper Response: {binascii.hexlify(response)}")
        return response

    def move(self, position, speed=255, force=255):
        """
        控制夹爪移动到任意指定位置（非阻塞优化）
        """
        if not (0 <= position <= 255):
            raise ValueError("目标位置必须在 0 到 255 的范围内！")
        if not (0 <= speed <= 255 or 0 <= force <= 255):
            raise ValueError("速度或力量值超出范围！")

        # 构造指令
        command = (
            b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00' +
            bytes([position, speed, force])
        )
        crc = self._calculate_crc(command)
        command += crc

        # 异步发送命令
        self.ser.write(command)

    def _calculate_crc(self, data):
        """
        计算 CRC 校验
        :param data: 待校验的字节数据
        :return: CRC16 校验码，返回低字节在前，高字节在后
        """
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    def get_gripper_status(self):
        """
        获取夹爪状态信息
        返回包含以下信息的字典:
        - gripper_status: 夹爪状态 (gACT, gGTO, gSTA)
        - object_status: 物体检测状态 (gOBJ)
        - fault_status: 故障状态 (gFLT)
        - position_request_echo: 位置请求回显 (gPR)
        - position: 当前位置 (gPO)
        - current: 当前电流 (gCU)
        """
        # 读取输入寄存器 (FC04) - 地址0x07D0开始，读取3个寄存器(6字节)
        command = b'\x09\x04\x07\xD0\x00\x03\xB1\xCE'
        # 读取响应 (11字节: 地址1 + 功能码1 + 字节数1 + 数据6 + CRC2)
        response = self.receive(command)

        if len(response) != 11:
            print("Error: Invalid response length")
            return None
        
        # 解析响应数据
        data = response[3:-2]  # 去掉地址、功能码、字节数和CRC
        # 按照文档中的寄存器映射解析数据
        status = {
            'gripper_status': {
                'gACT': (data[0] >> 0) & 0x01,  # 激活状态
                'gGTO': (data[0] >> 3) & 0x01,  # 动作状态
                'gSTA': (data[0] >> 4) & 0x03,  # 夹爪状态
                'gOBJ': (data[0] >> 6) & 0x03   # 物体检测状态
            },
            'fault_status': data[2],             # 故障状态
            'position_request_echo': data[3],    # 位置请求回显
            'position': data[4],                 # 当前位置
            'current': data[5]                   # 当前电流 (值×10 ≈ mA)
        }
        return status
    
    def get_gripper_extended_status(self):
        """
        获取详细的夹爪状态信息（包括人类可读的描述）
        """
        status = self.get_gripper_status()
        if status is None:
            return None
        
        # 详细状态描述
        gSTA_desc = {
            0x00: "Gripper is in reset (or automatic release) state",
            0x01: "Activation in progress",
            0x03: "Activation is completed"
        }
        gOBJ_desc = {
            0x00: "Fingers are in motion towards requested position. No object detected",
            0x01: "Fingers have stopped due to a contact while opening before requested position. Object detected opening",
            0x02: "Fingers have stopped due to a contact while closing before requested position. Object detected closing",
            0x03: "Fingers are at requested position. No object detected or object has been lost/dropped"
        }
        # 故障状态描述
        fault_desc = {
            0x00: "No fault (solid blue LED)",
            0x05: "Action delayed, the activation must be completed prior to performing the action",
            0x07: "The activation bit must be set prior to performing the action",
            0x08: "Maximum operating temperature exceeded",
            0x09: "No communication during at least 1 second",
            0x0A: "Under minimum operating voltage",
            0x0B: "Automatic release in progress",
            0x0C: "Internal fault",
            0x0D: "Activation fault",
            0x0E: "Overcurrent triggered",
            0x0F: "Automatic release completed"
        }
        
        # 添加描述信息
        gripper_status = status['gripper_status']
        gripper_status['gSTA_desc'] = gSTA_desc.get(gripper_status['gSTA'], "Unknown")
        gripper_status['gOBJ_desc'] = gOBJ_desc.get(gripper_status['gOBJ'], "Unknown")
        status['fault_desc'] = fault_desc.get(status['fault_status'], "Unknown fault")
        
        # 电流转换为实际值 (mA) 和计算扭矩
        status['current_mA'] = status['current'] * 10
        torque_constant = 0.02  # 假设电机力矩常数为 0.02 N·m / A
        status['motor_torque_Nm'] = (status['current_mA'] / 1000) * torque_constant
        
        return status

    def disconnect(self):
        """
        关闭串口连接
        """
        self.ser.close()

if __name__ == "__main__":
    gripper = RobotiqGripper(port='/dev/ttyUSB0')