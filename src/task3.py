#!/usr/bin/env python3 

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Task3:
    def __init__(self):
        rospy.init_node('task3_pt2', anonymous=True)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()

        # Initialising Class Variables
        self.turn = "No"
        self.minD_left = 0
        self.minD_front = 0
        self.maxD_front = 0 
        self.counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def scan_callback(self,scan_data):
        # Take measurements of distance from left wall (left side traversal method)
        # Needs to maintain that distance within 0.1m

        # Front left side view
        left_view = scan_data.ranges[30:60]
        left_array = np.array(left_view)
        self.minD_left = left_array.min()

        self.minA_left = left_array.argmin()

        # Front right side view
        right_view = scan_data.ranges[280:320]
        right_array = np.array(right_view)
        self.minD_right = right_array.min()

        # Front 30 degree view, for not hitting walls
        left_arc = scan_data.ranges[0:16]
        right_arc = scan_data.ranges[-15:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.minD_front = front_arc.min()

        # For seeing if a U-Turn is needed 
        wide_front_left = scan_data.ranges[0:91]
        wide_front_right = scan_data.ranges[-90:]
        wide_front_arc = np.array(wide_front_left[::-1] + wide_front_right[::-1])
        self.maxD_front = wide_front_arc.max()

        #Remove before submit
        #self.minInfo = f"Front: '{self.minD_front:.2f}',  Left: '{self.minD_left:.2f}', Right: '{self.minD_right:.2f}', maxFront '{self.maxD_front:.2f}'"

    def main_loop(self):
        rate = rospy.Rate(30)
        while not self.ctrl_c:
            # Room to move left so follow it 
            if (self.minD_left > 0.4) and (self.minD_front > 0.5):
                self.vel.linear.x = 0.26
                self.vel.angular.z = 1
            # Can't go left, but can follow left wall forward
            elif (self.minD_front > 0.5) and (self.minD_left > 0.3) and (self.minD_left < 0.4):
                self.vel.linear.x = 0.26
                self.vel.angular.z = 0.1
            # Can't go left or forward, but can go right
            elif (self.minD_left < 0.3) and (self.minD_front > 0.5):
                self.vel.linear.x = 0.26
                self.vel.angular.z = -0.6
            # Nowhere in front to go, does a 180 turn 
            elif (self.maxD_front < 0.65):
                self.vel.linear.x = 0.018
                self.vel.angular.z = math.pi/2
                self.pub.publish(self.vel)
                rospy.sleep(2)
            # If not, must be wall directly ahead
            else:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
                # Spins in the direction with most space to move
                if (self.minD_left > self.minD_right):
                    self.vel.angular.z = 1.5
                elif (self.minD_left < self.minD_right):
                    self.vel.angular.z = -1.5
            self.pub.publish(self.vel)        
        rate.sleep()
            
if __name__ == "__main__":
    node = Task3()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass