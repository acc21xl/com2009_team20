#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Task1:
    def __init__(self):
        self.x_init = None
        self.y_init = None
        self.yaw_init = None
        self.total_distance = 0.0
        self.counter = 10
        self.print_counter = 0 

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

        if self.yaw_init is None:
            orientation = pose.orientation 
            (roll, pitch, yaw_init) = euler_from_quaternion([orientation.x, 
                     orientation.y, orientation.z, orientation.w], 
                     'sxyz')
            self.yaw_init = yaw_init

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, 
                     orientation.y, orientation.z, orientation.w], 
                     'sxyz')
        
        if self.counter > 26:
            self.counter = 0
            one_loop = 360
            half_loop = one_loop/2
            degrees = (math.degrees(yaw-self.yaw_init)+ half_loop) % one_loop - half_loop
            self.print_counter += 1
            print(self.print_counter)
            print(f"x = {ongoing_x:.3f} (m), y = {ongoing_y:.3f} (m), theta_z = {degrees:.1f} (degrees)")
        else:
            self.counter += 1

        if self.x_init is not None and self.y_init is not None:
            distance = math.sqrt((ongoing_x - self.x_init)**2 + (ongoing_y - self.y_init)**2)
            self.total_distance += distance

        self.x_init = ongoing_x
        self.y_init = ongoing_y

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
                print("done")
                self.shutdownhook()


            self.pub.publish(self.vel)
            self.rate.sleep()


if __name__ == "__main__":
    node = Task1()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass