import PyTac3D
import time
import os
import subprocess
import threading
import argparse
import pickle


class TacSensor:

    def __init__(self, SNs, port):
        self.SN1, self.SN2 = SNs
        self.port = port
        self.P, self.N, self.D, self.F, self.Fr, self.Mr = None, None, None, None, None, None
        self.exceed_force = False

        # tac3d_core_path = '/home/robotics/Tac3D-SDK-v3.3.0/Tac3D-Core/linux-x86_64'
        # tac_core = os.path.join(tac3d_core_path, 'Tac3D')
        # config_path = os.path.join(tac3d_core_path, 'config')
        # start_sensor_1_cmd = f"{tac_core} -c {config_path}/{self.SN1} -i 127.0.0.1 -p {port}"
        # start_sensor_2_cmd = f"{tac_core} -c {config_path}/{self.SN2} -i 127.0.0.1 -p {port}"
        # subprocess.Popen(["gnome-terminal", "--", "bash", "-c", start_sensor_1_cmd])
        # subprocess.Popen(["gnome-terminal", "--", "bash", "-c", start_sensor_2_cmd])
        # subprocess.Popen([
        #     "gnome-terminal",
        #     "--tab", "--title=Sensor1", "--", "bash", "-c", start_sensor_1_cmd,
        #     "--tab", "--title=Sensor2", "--", "bash", "-c", start_sensor_2_cmd
        # ])

        self.save_data_dict = {self.SN1: {'tactile': [], 'deform': [], 'force': [], 'timestamps': []},
                               self.SN2: {'tactile': [], 'deform': [], 'force': [], 'timestamps': []},
                               'start_time': None}
        

    def Tac3DRecvCallback(self, frame, param):
        SN = frame['SN']
        timestamp = int(time.time() * 1000)
        self.P = frame.get('3D_Positions')
        self.D = frame.get('3D_Displacements')
        self.F = frame.get('3D_Forces')
        # self.N = frame.get('3D_Normals')
        self.Fr = frame.get('3D_ResultantForce')
        # self.Mr = frame.get('3D_ResultantMoment')

        if SN == self.SN1:
            self.save_data_dict[self.SN1]['tactile'].append(self.P)
            self.save_data_dict[self.SN1]['timestamps'].append(timestamp)
            self.save_data_dict[self.SN1]['deform'].append(self.D)
            self.save_data_dict[self.SN1]['force'].append(self.F)
        elif SN == self.SN2:
            self.save_data_dict[self.SN2]['tactile'].append(self.P)
            self.save_data_dict[self.SN2]['timestamps'].append(timestamp)
            self.save_data_dict[self.SN2]['deform'].append(self.D)
            self.save_data_dict[self.SN2]['force'].append(self.F)
        
        if abs(self.Fr[0, -1]) > 10:
            self.exceed_force = True

    def wait_for_enter(self, stop_event):
        """Thread function to wait for Enter key press"""
        input()  # This will block until Enter is pressed
        stop_event.set()
    
    def record_data(self):
        self.stop_event = threading.Event()
        input_thread = threading.Thread(target=self.wait_for_enter, args=(self.stop_event,))
        input_thread.daemon = True
        input_thread.start()

        self.tac3d = PyTac3D.Sensor(recvCallback=self.Tac3DRecvCallback, port=self.port, maxQSize=5, callbackParam = 'test param')
        self.tac3d.waitForFrame(SN=self.SN1)
        self.tac3d.waitForFrame(SN=self.SN2)
        time.sleep(2)
        self.tac3d.calibrate(SN=self.SN1)
        self.tac3d.calibrate(SN=self.SN2)
        time.sleep(2)

        self.start_time = int(time.time() * 1000)
        print('start to record tactile data...')
        print('Press Enter to stop recording...')
        
        while True:
            if self.stop_event.is_set():
                break
        
        self.save_data_dict['start_time'] = self.start_time

        return self.save_data_dict
    
    def get_force(self):
        self.tac3d = PyTac3D.Sensor(recvCallback=self.Tac3DRecvCallback, port=self.port, maxQSize=5, callbackParam = 'test param')
        self.tac3d.waitForFrame(SN=self.SN1)
        self.tac3d.waitForFrame(SN=self.SN2)
        self.tac3d.calibrate(SN=self.SN1)
        self.tac3d.calibrate(SN=self.SN2)
        while True:
            pass
            

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_dir", type=str, default="/home/robotics/data_save")
    parser.add_argument("--traj_number", type=int, default=1)
    args = parser.parse_args()

    tacsensor = TacSensor(SNs=['DL1-GWM0001', 'DL1-GWM0002'], port=9988)
    save_data_dict = tacsensor.record_data()

    # save_path = os.path.join(args.root_dir, str(args.traj_number).zfill(4), 'tactile')
    # os.makedirs(save_path, exist_ok=True)

    # with open(os.path.join(save_path, 'tactile.pkl'), 'wb') as f:
    #     pickle.dump(save_data_dict, f)

