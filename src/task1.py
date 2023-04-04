#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Task1:
    def __init__(self):
        self.prev_x = None
        self.prev_y = None
        self.distance_covered = 0.0
        self.counter = 0

        rospy.init_node('task1', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True


    def odometry_callback(self, odom):
        curr_x = odom.pose.pose.position.x
        curr_y = odom.pose.pose.position.y
        curr_z = odom.pose.pose.position.z

        pose = odom.pose.pose
        orientation = pose.orientation 

        (roll, pitch, yaw) = euler_from_quaternion([curr_x, 
                     curr_y, curr_z, orientation.w], 
                     'sxyz')

        if self.counter > 10:
            self.counter = 0
            degrees = (yaw * 180)/math.pi
            print(f"x = {curr_x:.3f} (m), y = {curr_y:.3f} (m), theta_z = {degrees:.2f} (degrees)")
        else:
            self.counter += 1

        #if self.prev_x is not None and self.prev_y is not None:
        #    distance = math.sqrt((curr_x - self.prev_x)**2 + (curr_y - self.prev_y)**2)
        #    self.distance_covered += distance

        #self.prev_x = curr_x
        #self.prev_y = curr_y

    def main_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not self.ctrl_c:
            if self.distance_covered <= (2 * math.pi * 0.5):
                self.vel.linear.x = 0.115
                self.vel.angular.z = 0.23
            elif self.distance_covered <= 2 * (2 * math.pi * 0.5):
                self.vel.linear.x = 0.115
                self.vel.angular.z = -0.23

            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0


            self.pub.publish(self.vel)

            rate.sleep()
    

if __name__ == "__main__":
    node = Task1()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
