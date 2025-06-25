import serial
import time
import binascii

""" Gripper Controll Class """

class GripperController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1,
                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout,
                                 parity=parity, stopbits=stopbits, bytesize=bytesize)
        # self.activate()

    def activate(self):
        self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
        data_raw = self.ser.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        print("activate response ", data)
        time.sleep(0.01)

    def close_gripper(self):
        print("closing the gripper")
        self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x64\x14\x14\x3D\x79')
        # self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x32\x0A\x0A\x54\xC1')
        data_raw = self.ser.readline()
        # print(data_raw)
        data = binascii.hexlify(data_raw)
        print("close response ", data)
        time.sleep(2)

    def open_gripper(self):
        print("opening the gripper")
        """ force control """
        # full force
        # self.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29')
        # half force
        # self.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\x80\x03\xC9')
        
        """ position control (different length), (0, 255), 0 means fully closed """
        # 0 / 255
        self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19')
        # 50 / 255
        # self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x32\xFF\xFF\xD3\xD6')
        # 70 / 255
        # self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x46\xFF\xFF\x93\xCC')
        # 100 / 255
        # self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x64\xFF\xFF\x33\xC6')
        # 150 / 255
        # self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x96\xFF\xFF\x92\x35')
        # 170 / 255
        # self.ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xAA\xFF\xFF\x52\x39')
        
        data_raw = self.ser.readline()
        # print(data_raw)
        data = binascii.hexlify(data_raw)
        print("open gripper respons应 ", data)
        time.sleep(2)

if __name__ == "__main__":
    gripper = GripperController()
    # activate the gripper
    # gripper.activate()
    # gripper.open_gripper()
    # gripper.close_gripper()
