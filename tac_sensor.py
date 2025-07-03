import PyTac3D
import time
import os
import argparse
import subprocess
import pickle
import numpy as np


def get_timestamp():
    return str(int(time.time() * 1000 - 100))  # 100ms延迟

def save_to_pickle(data, save_path):
    """
    保存数据到指定的子文件夹中。
    Args:
        data (dict): 要保存的数据。
        base_folder (str): 存放所有实验数据的主文件夹路径。
        traj_number (str): 当前实验编号，用于创建子文件夹。
    """
    os.makedirs(save_path, exist_ok=True)
    try:
        
        # 自动生成文件名
        file_name = f"{get_timestamp()}.pkl"
        file_path = os.path.join(save_path, file_name)

        # 保存数据到 pickle 文件
        with open(file_path, 'wb') as f:
            pickle.dump(data, f)

    except Exception as e:
        print(f"Error saving data to {save_path}: {e}")

class Tacsensors:
    def __init__(self, SNs, port):
        self.SNs = SNs
        SN1, SN2 = SNs
        self.first_frame_1 = None
        self.first_frame_2 = None
        
        # start tac3d core
        tac3d_core_path = '/home/robotics/workspace/Tac3D-SDK-v3.3.0/Tac3D-Core/linux-x86_64'
        tac_core = os.path.join(tac3d_core_path, 'Tac3D')
        config_path = os.path.join(tac3d_core_path, 'config')
        start_sensor_1_cmd = f"{tac_core} -c {config_path}/{SN1} -i 127.0.0.1 -p {port}"
        start_sensor_2_cmd = f"{tac_core} -c {config_path}/{SN2} -i 127.0.0.1 -p {port}"
        
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", start_sensor_1_cmd])
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", start_sensor_2_cmd])
        # subprocess.Popen([
        #     "gnome-terminal",
        #     "--tab", "--title=Sensor1", "--", "bash", "-c", start_sensor_1_cmd,
        #     "--tab", "--title=Sensor2", "--", "bash", "-c", start_sensor_2_cmd
        # ])

        self.tac3d = PyTac3D.Sensor(port=port, maxQSize=10)
        # Wait for the Tac3D sensor to start and send data
        self.tac3d.waitForFrame(SN=SN1)
        self.tac3d.waitForFrame(SN=SN2)
        time.sleep(2)
        
        self.tac3d.calibrate(SN=SN1)
        self.tac3d.calibrate(SN=SN2)
        time.sleep(1)

    def run(self):
        SN1, SN2 = self.SNs
        frame_1 = self.tac3d.getFrame(SN=SN1)
        frame_2 = self.tac3d.getFrame(SN=SN2)
        
        return frame_1, frame_2
    
    def record_data(self, save_path):
        SN1, SN2 = self.SNs
        SN1_save = os.path.join(save_path, SN1 + '_r')
        SN2_save = os.path.join(save_path, SN2 + '_l')
        while True:
            frame_1, frame_2 = self.run()
            if frame_1 is not None:
                current_frame_1 = frame_1['3D_Positions']
                if self.first_frame_1 is None:
                    self.first_frame_1 = current_frame_1
                right_tactile = np.array([self.first_frame_1, current_frame_1])
                save_to_pickle(right_tactile, SN1_save)
            
            if frame_2 is not None:
                current_frame_2 = frame_2['3D_Positions']
                if self.first_frame_2 is None:
                    self.first_frame_2 = current_frame_2
                left_tactile = np.array([self.first_frame_2, current_frame_2])
                save_to_pickle(left_tactile, SN2_save)
    
    def get_force(self):
        fn_l = None
        fn_r = None
        frame_1, frame_2 = self.run()
        if frame_1 is not None:
            force = frame_1['3D_ResultantForce']
            fx_r = force[0, 0]
            fy_r = force[0, 1]
            fn_r = force[0, 2]
        if frame_2 is not None:
            force = frame_2['3D_ResultantForce']
            fx_l = force[0, 0]
            fy_l = force[0, 1]
            fn_l = force[0, 2]
        return fn_l, fn_r

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_dir", type=str, default="/home/robotics/workspace/xx_/real/scripts/umi/data_save")
    parser.add_argument("--traj_number", type=int, default=1)
    args = parser.parse_args()

    tacsensors = Tacsensors(SNs=['DL1-GWM0001', 'DL1-GWM0002'], port=9988)

    save_path = os.path.join(args.root_dir, str(args.traj_number).zfill(4), 'tactile')
    os.makedirs(save_path, exist_ok=True)
    
    tacsensors.record_data(save_path)
    # while True:
    #     fn_l, fn_r = tacsensors.get_force()
    #     if fn_l is not None and fn_r is not None:
    #         print(fn_l, fn_r)


    