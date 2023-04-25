#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Task1:
    def _init_(self):
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.total_distance = 0.0
        self.counter = 10

        rospy.init_node('task1', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.callback_function)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)  # hz

        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True


    def callback_function(self, odom):
        ongoing_x = odom.pose.pose.position.x
        ongoing_y = odom.pose.pose.position.y
        pose = odom.pose.pose
        orientation = pose.orientation 

        if self.initial_yaw is None:
            orientation = pose.orientation 
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                     orientation.y, orientation.z, orientation.w], 
                     'sxyz')
            self.initial_yaw = yaw

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                     orientation.y, orientation.z, orientation.w], 
                     'sxyz')

        if self.counter > 10:
            self.counter = 0
            degrees = (math.degrees(yaw-self.initial_yaw)+180) % 360 - 180
            print(f"x = {ongoing_x:.3f} (m), y = {ongoing_y:.3f} (m), theta_z = {degrees:.1f} (degrees)")
        else:
            self.counter += 1

        if self.initial_x is not None and self.initial_y is not None:
            distance = math.sqrt((ongoing_x - self.initial_x)*2 + (ongoing_y - self.initial_y)*2)
            self.total_distance += distance

        self.initial_x = ongoing_x
        self.initial_y = ongoing_y

    def main_loop(self):
        
        while not self.ctrl_c:
            if self.total_distance <= (2 * math.pi * 0.5):
                self.vel.linear.x = 0.115
                self.vel.angular.z = 0.23
            elif self.total_distance <= 2 * (2 * math.pi * 0.5):
                self.vel.linear.x = 0.115
                self.vel.angular.z = -0.23

            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0


            self.pub.publish(self.vel)

            self.rate.sleep()
    

if _name_ == "_main_":
    node = Task1()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass