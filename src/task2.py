#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Square():
    def __init__(self):
        node_name = "task2"
        rospy.init_node(node_name, anonymous=True)
        self.ctrl_c = False
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
        self.front = 0
        self.rear = 0
        self.rate = rospy.Rate(10)  # hz
        self.vel = Twist()
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    def laserscan_cb(self, scan_data):
        self.rear = np.array(scan_data.ranges[120:230]).min()
        left_arc = scan_data.ranges[0:25]
        right_arc = scan_data.ranges[-25:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])    
        self.front = front_arc.min()

    def main_loop(self):
        while not self.ctrl_c:
            self.rear_precaution()
            if (self.front > 0.4):
                self.vel.linear.x = 0.24
                self.vel.angular.z = 0.2
            if (self.front < 0.5):
                self.vel.linear.x = 0.0
                self.vel.angular.z = -0.6 
            self.pub.publish(self.vel)
            self.rate.sleep()

    def rear_precaution(self):
        while (self.rear < 0.2):
            if (self.front < 0.4):
                self.vel.linear.x = 0.05
                self.vel.angular.z = 0.0
            else:
                self.vel.linear.x = 0.1
                self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            self.rate.sleep()
            
if __name__ == "__main__":
    node = Square()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass




