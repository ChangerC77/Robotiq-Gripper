#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped

class TactileForceSubscriber:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('tactile_force_subscriber', anonymous=True)
        
        # 订阅两个传感器的力数据话题
        rospy.Subscriber('/tac_sensor/force_SN1', WrenchStamped, self.force_callback_SN1)
        rospy.Subscriber('/tac_sensor/force_SN2', WrenchStamped, self.force_callback_SN2)
        
        # 初始化力数据变量
        self.force_SN1 = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.force_SN2 = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 设置打印频率(Hz)
        self.print_rate = rospy.Rate(10)
        
    def force_callback_SN1(self, msg):
        """SN1传感器的力数据回调函数"""
        self.force_SN1['x'] = msg.wrench.force.x
        self.force_SN1['y'] = msg.wrench.force.y
        self.force_SN1['z'] = msg.wrench.force.z
        
    def force_callback_SN2(self, msg):
        """SN2传感器的力数据回调函数"""
        self.force_SN2['x'] = msg.wrench.force.x
        self.force_SN2['y'] = msg.wrench.force.y
        self.force_SN2['z'] = msg.wrench.force.z
        
    def print_force_data(self):
        """打印力数据"""
        while not rospy.is_shutdown():
            # 清空终端行
            print('\r', end='')
            
            # 打印SN1数据
            print(f"SN1 - X:{self.force_SN1['x']:.3f}N, ", end='')
            print(f"Y:{self.force_SN1['y']:.3f}N, ", end='')
            print(f"Z:{self.force_SN1['z']:.3f}N | ", end='')
            
            # 打印SN2数据
            print(f"SN2 - X:{self.force_SN2['x']:.3f}N, ", end='')
            print(f"Y:{self.force_SN2['y']:.3f}N, ", end='')
            print(f"Z:{self.force_SN2['z']:.3f}N", end='')
            
            # 刷新输出缓冲区
            sys.stdout.flush()
            
            # 按照设定频率休眠
            self.print_rate.sleep()

if __name__ == '__main__':
    try:
        subscriber = TactileForceSubscriber()
        subscriber.print_force_data()
    except rospy.ROSInterruptException:
        pass