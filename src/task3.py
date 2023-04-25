#!/usr/bin/env python3 

#Need Lidar
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
#Odom
#Twist

class Task3:
    def __init__(self):
        rospy.init_node('task3', anonymous=True)
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
        self.turn = "No"
        self.counter = 0
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def scan_callback(self,scan_data):
        front_left_view = scan_data.ranges[0:71]
        front_right_view = scan_data.ranges[-70:]
        left_view = scan_data.ranges[40:70]
        left_array = np.array(left_view)
        right_view = scan_data.ranges[280:320]
        right_array = np.array(right_view)
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        full_front_arc = np.array(front_left_view[::-1] + front_right_view[::-1])
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        maxD_front = full_front_arc.max()
        minD_front = front_arc.min()
        self.minD_left = left_array.min()
        self.minD_right = right_array.min()
        if (minD_front < 0.25):
            self.turn = "noCrash"
        elif ((self.minD_left > 0.45 or self.minD_right < 0.25) and minD_front > 0.30):
            self.turn = "Left"
        elif ((self.minD_left < 0.25 or self.minD_right > 0.5) and minD_front > 0.30):
            self.turn = "Right"
        elif (maxD_front < 0.65):
            self.turn = "uTurn"
        elif (minD_front < 0.6 and self.minD_right > 0.4):
            self.turn = "Right"
        else:
            self.turn = "No"
        self.minInfo = f"Front: '{minD_front:.2f}',  Left: '{self.minD_left:.2f}', Right: '{self.minD_right:.2f}', MaxFront: '{maxD_front:.2f}'"

    def odometry_callback(self,odom):
        pose = odom.pose.pose
        orientation = pose.orientation

        curr_x = pose.position.x
        curr_y = pose.position.y
        curr_z = pose.position.z

        (roll, pitch, yaw) = euler_from_quaternion([curr_x, 
                     curr_y, curr_z, orientation.w], 
                     'sxyz')
        #idk if we need you g x
        if self.counter > 15:
            self.counter = 0
            print(f"'{self.minInfo}'")
            print(f"he is turning'{self.turn}'")
        else:
            self.counter += 1

    def main_loop(self):
        rate = rospy.Rate(30)
        while not self.ctrl_c:
            if self.turn == "Left":
                self.vel.linear.x = 0.20
                if self.minD_right > 0.25:
                    self.vel.angular.z = 1.5
                elif self.minD_left < 0.9:
                    self.vel.angular.z = self.minD_left * 2
                else:
                    self.vel.angular.z = 1
            elif self.turn == "Right":
                self.vel.linear.x = 0.20
                if self.minD_left > 0.25:
                    self.vel.angular.z = -1.5
                elif self.minD_right < 0.9:
                    self.vel.angular.z = self.minD_right * -2
                else:
                    self.vel.angular.z = 1
            elif self.turn == "noCrash":
                self.vel.linear.x = -0.15
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
                rospy.sleep(1)
                self.pub.publish(Twist())
                rospy.sleep(1)
            elif self.turn == "uTurn":
                self.vel.linear.x = 0
                self.vel.angular.z = math.pi/2
                self.pub.publish(self.vel)
                rospy.sleep(2)
                self.pub.publish(Twist())
                rospy.sleep(2)
            else:
                self.vel.linear.x = 0.25
                self.vel.angular.z = 0
            self.pub.publish(self.vel)
            rate.sleep()
        #Responding to scan data, setting Twist data, turning whatever
        

if __name__ == "__main__":
    node = Task3()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
