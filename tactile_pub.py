#!/usr/bin/env python
import os
import sys
import rospy
from std_msgs.msg import Float32
import threading

# 动态添加路径
collect_path = os.path.expanduser("~/umi")
sys.path.append(collect_path)
from tac_sensor_callback import TacSensor

class TactileSensorNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('tactile_sensor_publisher', anonymous=True)
        
        # 创建Publisher，发布到"tactile_force"话题，消息类型为Float32
        self.force_pub = rospy.Publisher('tactile_force', Float32, queue_size=10)
        
        # 设置发布频率 (Hz)
        self.rate = rospy.Rate(100)  # 100Hz
        
        # 初始化传感器
        self.tacsensor = TacSensor(SNs=['DL1-GWM0001', 'DL1-GWM0002'], port=9988)
        
        # 启动传感器数据读取线程
        self.input_thread = threading.Thread(target=self.tacsensor.get_force, daemon=True)
        self.input_thread.start()
    
    def publish_data(self):
        while not rospy.is_shutdown():
            if self.tacsensor.Fr is not None:
                # 获取力值并取绝对值
                force_value = abs(self.tacsensor.Fr[0, -1])
                
                # 创建并发布消息
                force_msg = Float32()
                force_msg.data = force_value
                self.force_pub.publish(force_msg)
                
                # 可选：打印日志
                rospy.loginfo_once("Publishing tactile force data: %.6f" % force_value)
            
            # 按照设定的频率休眠
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TactileSensorNode()
        node.publish_data()
    except rospy.ROSInterruptException:
        pass