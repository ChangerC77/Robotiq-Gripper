import struct

def calculate_crc(data):
    """
    计算CRC16 MODBUS校验码
    :param data: 待校验的字节数据
    :return: CRC16校验码, 返回低字节在前, 高字节在后的形式
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
    # 返回低字节在前，高字节在后
    return struct.pack('<H', crc)

def build_protocol(position, speed, force):
    """
    构建通讯协议数据帧
    :param position: 位置(0-255)
    :param speed: 速度(0-255)
    :param force: 力(0-255)
    :return: 完整的通讯协议字节帧
    """
    # 检查输入的范围是否符合要求
    if not (0 <= position <= 255 and 0 <= speed <= 255 and 0 <= force <= 255):
        raise ValueError("位置、速度和力的值必须在0-255范围内!")

    # 固定部分的数据帧
    frame = bytearray([0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x09, 0x00, 0x00])

    # 添加位置、速度和力
    frame += bytearray([position, speed, force])

    # 计算CRC16校验码
    crc = calculate_crc(frame)

    # 添加CRC到数据帧
    frame += crc

    return frame

def format_output(frame):
    """
    格式化输出字节帧为 ser.write(b'...')
    :param frame: 字节帧
    :return: 格式化字符串
    """
    # 将字节帧转换为大写的十六进制字符串
    hex_string = ''.join(f'\\x{byte:02X}' for byte in frame)
    return f"ser.write(b'{hex_string}')"

# 示例使用
if __name__ == "__main__":
    # 输入位置、速度和力
    print("255为夹爪全力闭合全速状态")
    position = int(input("请输入位置(0-255): "))
    speed = int(input("请输入速度(0-255): "))
    force = int(input("请输入力(0-255): "))

    # 生成通讯协议数据帧
    try:
        protocol_frame = build_protocol(position, speed, force)
        # 格式化输出并打印
        output = format_output(protocol_frame)
        print(output)
    except ValueError as e:
        print(e)