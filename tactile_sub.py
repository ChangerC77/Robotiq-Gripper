#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

class TactileSubscriber:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('tactile_subscriber', anonymous=True)
        
        # 订阅tactile_force话题
        rospy.Subscriber('tactile_0/force', Float32, self.tactile_0_callback)
        rospy.Subscriber('tactile_1/force', Float32, self.tactile_1_callback)
        
        rospy.loginfo("触觉力数据订阅器已启动，等待数据...")
    
    def tactile_0_callback(self, msg):
        """简单的力数据回调函数"""
        # 直接打印接收到的力值
        rospy.loginfo("tactile_0 force: %.6f" % msg.data)
    
    def tactile_1_callback(self, msg):
        """简单的力数据回调函数"""
        # 直接打印接收到的力值
        rospy.loginfo("tactile_1 force: %.6f" % msg.data)

if __name__ == '__main__':
    try:
        subscriber = TactileSubscriber()
        subscriber.tactile_0_callback
        subscriber.tactile_1_callback
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        pass