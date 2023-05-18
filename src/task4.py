#!/usr/bin/env python3
import argparse
import time
from turtle import right
import rospy
import numpy as np
import math
from pathlib import Path 
import roslaunch

from math import pi, sqrt, pow
# For creating images
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class Task4():
    def __init__(self):  
        # a flag if this node has just been launched
        self.startup = True

        self.turn = False

        self.node_name = "task4"        
        self.colour_faced = "None"
        self.lower_hsv = (0,100,100)
        self.upper_hsv = (100,255,255)

        self.m00 = 0
        self.m00_min = 7500000

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.subScan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Publisher node for movement
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        # Create function for creating a mask and checking if colour is valid, call for each colour until matches         

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()
        self.counter = 0

        self.base_image_path = Path.home().joinpath("catkin_ws/src/com2009_team20/snaps/")
        self.base_image_path.mkdir(parents=True, exist_ok=True) 
        self.pic_taken = False

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {self.node_name} node has been initialised...")

    def create_mask(self,upper_hsv,lower_hsv, cv_img):
        m00_min = 10000
        height, width, _ = cv_img.shape
        crop_width = width - 800    
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img,lower_hsv, upper_hsv)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        m00 = m['m00']
        cy = m['m10'] / (m['m00'] + 1e-5)

        if m00 > m00_min:
            return True
        else : 
            return False
        
    def shutdownhook(self):
        self.pub.publish(Twist())
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            self.cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, _ = self.cv_img.shape
        crop_width = width - 800
        
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = self.cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img,self.lower_hsv, self.upper_hsv)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)
        #print(self.cy)
        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def scan_callback(self,scan_data):

        # Front Left
        left_view = scan_data.ranges[30:60]
        left_array = np.array(left_view)
        self.minD_left = left_array.min()

        self.minA_left = left_array.argmin()

        # Front right side view
        right_view = scan_data.ranges[300:330]
        right_array = np.array(right_view)
        self.minD_right = right_array.min()

        # Front 30 degree view, for not hitting walls
        left_arc = scan_data.ranges[0:26]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.minD_front = front_arc.min()

        # For seeing if a U-Turn is needed 
        wide_front_left = scan_data.ranges[0:56]
        wide_front_right = scan_data.ranges[-55:]
        wide_front_arc = np.array(wide_front_left[::-1] + wide_front_right[::-1])
        self.maxD_front = wide_front_arc.max()
    
    def main_loop(self):
        self.target = ""
        while not self.ctrl_c:

            if (self.colour_faced == "None"):
                rospy.sleep(1)
                self.vel.linear.x = 0
                self.vel.angular.z = math.pi/2
                self.pub.publish(self.vel)
                rospy.sleep(1)
                while (self.colour_faced == "None"):
                    blue_lower = (115, 50, 100)
                    blue_upper = (130, 255, 255)

                    green_lower = (50, 70, 100)
                    green_upper = (70,255,255)

                    yellow_lower = (25,50,100)
                    yellow_upper = (40, 255, 255)

                    turq_lower = (85, 200, 100)
                    turq_upper = (100, 255, 255) 

                    red_lower = (0, 50, 100)
                    red_upper = (10, 255, 255)

                    purple_lower = (145, 200, 100)
                    purple_upper = (165, 255, 255)

                    if (self.create_mask(blue_upper,blue_lower, self.cv_img) == True):
                        self.colour_faced = "Blue"
                        self.lower_hsv = (120,50,100)
                        self.upper_hsv = (135,255,255)
                    elif (self.create_mask(green_upper,green_lower, self.cv_img) == True):
                        self.colour_faced = "Green"
                        self.lower_hsv = (50, 70, 100)
                        self.upper_hsv = (70, 255, 255)
                    elif (self.create_mask(yellow_upper,yellow_lower, self.cv_img) == True):
                        self.colour_faced = "Yellow"
                        self.lower_hsv = (25,50,100)
                        self.upper_hsv = (40, 255, 255)
                    elif (self.create_mask(turq_upper,turq_lower, self.cv_img) == True):
                        self.colour_faced = "Turquoise"
                        self.lower_hsv = (85, 200, 100)
                        self.upper_hsv = (100, 255, 255)
                    elif (self.create_mask(red_upper,red_lower, self.cv_img) == True):
                        self.colour_faced = "Red"
                        self.lower_hsv = (0, 50, 100)
                        self.upper_hsv = (10, 255, 255)
                    elif (self.create_mask(purple_upper,purple_lower, self.cv_img) == True):
                        self.colour_faced = "Purple"
                        self.lower_hsv = (145, 200, 100)
                        self.upper_hsv = (165, 255, 255)

                print(f"SEARCH INITIATED: The target beacon colour is {self.colour_faced}.")

                self.vel.linear.x = 0
                self.vel.angular.z = -1*(math.pi/2)
                self.pub.publish(self.vel)
                rospy.sleep(1)
                self.pub.publish(Twist())
                rospy.sleep(2)

            if (self.colour_faced != "None" and self.m00 > self.m00_min):
                self.vel.linear.x = 0 
                self.vel.angular.z = 0 
                self.pub.publish()
                while self.cy < 550 or self.cy > 570:
                    if self.cy < 450:
                        self.vel.angular.z = 0.5
                    elif self.cy > 670 :
                        self.vel.angular.z = -0.5
                    elif self.cy < 670 and self.cy > 450:
                        if self.cy < 550:
                            self.vel.angular.z = 0.05
                        elif self.cy > 570:
                            self.vel.angular.z = - 0.05
                    self.pub.publish(self.vel)
                print("TARGET DETECTED: Beaconing initiated.")
                while (self.minD_front > 0.35):
                    if (self.minD_left < 0.3): 
                        self.vel.angular.z = -0.1
                    elif (self.minD_right < 0.3):
                        self.vel.angular.z = 0.1
                    else:
                        if self.cy < 550:
                            self.vel.angular.z = 0.1
                        elif self.cy > 570:
                            self.vel.angular.z = - 0.1 
                        
                    self.vel.linear.x = 0.15
                    self.pub.publish(self.vel)
                self.vel.linear.x = 0 
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
                print("BEACONING COMPLETE: The robot has now stopped.")
                self.shutdownhook()
                #TARGET REACHED END PROGRAM 
            
            # Room to move left so follow it 
            if (self.minD_left > 0.6) and (self.minD_front > 0.5):
                self.vel.linear.x = 0.23
                self.vel.angular.z = 1
            # Can't go left, but can follow left wall forward
            elif (self.minD_front > 0.5) and (self.minD_left > 0.5) and (self.minD_left < 0.6):
                self.vel.linear.x = 0.23
                self.vel.angular.z = 0.1
            # Can't go left or forward, but can go right
            elif (self.minD_left < 0.5) and (self.minD_front > 0.5):
                self.vel.linear.x = 0.23
                self.vel.angular.z = -0.6
            # Nowhere in front to go, does a 180 turn 
            elif (self.maxD_front < 0.9):
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
                rospy.sleep(0.3)
            self.pub.publish(self.vel)        
        self.rate.sleep()

if __name__ == "__main__":
    node = Task4()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass