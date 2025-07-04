#!/usr/bin/env python
import os
import sys
import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
import threading

# 动态添加路径
collect_path = os.path.expanduser("~/umi")
sys.path.append(collect_path)
from collect_tactile_callback import TacSensor

class TactileSensorNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('tactile_sensor_publisher', anonymous=True)
        
        # 创建Publisher (保持原有不变)
        self.force_pub_0 = rospy.Publisher('tactile_0/force', Float32, queue_size=10)
        self.tactile_pub_0 = rospy.Publisher('tactile_0/tactile_array', Float32MultiArray, queue_size=10)
        self.deform_pub_0 = rospy.Publisher('tactile_0/deform_array', Float32MultiArray, queue_size=10)
        
        self.force_pub_1 = rospy.Publisher('tactile_1/force', Float32, queue_size=10)
        self.tactile_pub_1 = rospy.Publisher('tactile_1/tactile_array', Float32MultiArray, queue_size=10)
        self.deform_pub_1 = rospy.Publisher('tactile_1/deform_array', Float32MultiArray, queue_size=10)
        
        # 新增时间相关的Publisher
        self.start_time_pub = rospy.Publisher('tactile/start_time', Float32, queue_size=10)
        self.timestamp_pub_0 = rospy.Publisher('tactile_0/timestamp', Float32, queue_size=10)
        self.timestamp_pub_1 = rospy.Publisher('tactile_1/timestamp', Float32, queue_size=10)
        
        # 设置发布频率 (Hz)
        self.rate = rospy.Rate(100)  # 100Hz
        
        # 初始化传感器
        self.tacsensor = TacSensor(SNs=['DL1-GWM0001', 'DL1-GWM0002'], port=9988, save_path="/tmp")
        
        # 启动传感器数据读取线程
        self.input_thread = threading.Thread(target=self.tacsensor.record_data, daemon=True)
        self.input_thread.start()
        
        # 保存start_time的本地副本
        self.start_time_value = None
    
    def publish_data(self):
        while not rospy.is_shutdown():
            # 持续发布start_time（每次循环都发布）
            if self.tacsensor.save_data_dict['start_time'] is not None and self.start_time_value is None:
                self.start_time_value = self.tacsensor.save_data_dict['start_time']
            
            if self.start_time_value is not None:
                start_time_msg = Float32()
                start_time_msg.data = self.start_time_value
                self.start_time_pub.publish(start_time_msg)
            
            # 发布传感器0的数据（保持原有逻辑）
            if self.tacsensor.save_data_dict['DL1-GWM0001']['force']:
                latest_idx = len(self.tacsensor.save_data_dict['DL1-GWM0001']['force']) - 1
                
                # 发布力数据
                force_value = np.sum(self.tacsensor.save_data_dict['DL1-GWM0001']['force'][latest_idx][:, -1])
                force_msg = Float32()
                force_msg.data = force_value
                self.force_pub_0.publish(force_msg)
                
                # 发布触觉阵列
                tactile_msg = Float32MultiArray()
                tactile_msg.data = self.tacsensor.save_data_dict['DL1-GWM0001']['tactile'][latest_idx].flatten()
                self.tactile_pub_0.publish(tactile_msg)
                
                # 发布形变阵列
                deform_msg = Float32MultiArray()
                deform_msg.data = self.tacsensor.save_data_dict['DL1-GWM0001']['deform'][latest_idx].flatten()
                self.deform_pub_0.publish(deform_msg)
                
                # 发布时间戳（新增）
                timestamp_msg = Float32()
                timestamp_msg.data = self.tacsensor.save_data_dict['DL1-GWM0001']['timestamps'][latest_idx]
                self.timestamp_pub_0.publish(timestamp_msg)
            
            # 发布传感器1的数据（保持原有逻辑）
            if self.tacsensor.save_data_dict['DL1-GWM0002']['force']:
                latest_idx = len(self.tacsensor.save_data_dict['DL1-GWM0002']['force']) - 1
                
                # 发布力数据
                force_value = np.sum(self.tacsensor.save_data_dict['DL1-GWM0002']['force'][latest_idx][:, -1])
                force_msg = Float32()
                force_msg.data = force_value
                self.force_pub_1.publish(force_msg)
                
                # 发布触觉阵列
                tactile_msg = Float32MultiArray()
                tactile_msg.data = self.tacsensor.save_data_dict['DL1-GWM0002']['tactile'][latest_idx].flatten()
                self.tactile_pub_1.publish(tactile_msg)
                
                # 发布形变阵列
                deform_msg = Float32MultiArray()
                deform_msg.data = self.tacsensor.save_data_dict['DL1-GWM0002']['deform'][latest_idx].flatten()
                self.deform_pub_1.publish(deform_msg)
                
                # 发布时间戳（新增）
                timestamp_msg = Float32()
                timestamp_msg.data = self.tacsensor.save_data_dict['DL1-GWM0002']['timestamps'][latest_idx]
                self.timestamp_pub_1.publish(timestamp_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TactileSensorNode()
        node.publish_data()
    except rospy.ROSInterruptException:
        pass