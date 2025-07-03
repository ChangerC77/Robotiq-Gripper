#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

class TactileForceSubscriber:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('tactile_force_subscriber', anonymous=True)
        
        # 订阅tactile_force话题
        rospy.Subscriber('tactile_force', Float32, self.force_callback)
        
        rospy.loginfo("触觉力数据订阅器已启动，等待数据...")
    
    def force_callback(self, msg):
        """简单的力数据回调函数"""
        # 直接打印接收到的力值
        rospy.loginfo("接收到力值: %.6f" % msg.data)

if __name__ == '__main__':
    try:
        subscriber = TactileForceSubscriber()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        pass