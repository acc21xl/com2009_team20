#!/usr/bin/env python3
import argparse
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
    def odometry_callback(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables (so that we can
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = abs(yaw)

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.turn = True
            self.theta_z0 = self.theta_z

    def __init__(self):  
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() (to switch between 
        # turning and moving forwards)
        self.turn = False

        self.node_name = "task4" 
        
        # How node takes arguments for which colour to look for
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{self.node_name}' node.")
        cli.add_argument("colour", metavar="COL", default="Black",help="The name of a colour(Blue/Red/Yellow?Green)"
        )

        args, self.colour_arg = cli.parse_known_args(rospy.myargv()[1:])
       

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.subScan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Publisher node for movement
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        self.lower_hsv = (0,0,100)
        self.upper_hsv = (175,255,255)
        self.colour_faced = ""

        if self.colour_arg[0] == "blue":
            self.lower_hsv = (115, 50, 100)
            self.upper_hsv = (160, 255, 255)
        elif self.colour_arg[0] == "red":
            self.lower_hsv = (0, 50, 100)
            self.upper_hsv = (10, 255, 255)
        elif self.colour_arg[0] == "yellow":
            self.lower_hsv = (25,50,100)
            self.upper_hsv = (40,255,255)
        elif self.colour_arg[0] == "green":
            self.lower_hsv = (50, 70, 100)
            self.upper_hsv = (70, 255, 255)
        else:
            print("Invalid Colour Choice")
        
        print(f"The Color Now Faced Is {self.colour_arg[0]}")
        # print(f"TASK 5 BEACON: The target is {self.colour_arg[0]}")
        

        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()
        self.counter = 0
        self.m00 = 0
        self.m00_min = 10000
        self.base_image_path = Path.home().joinpath("/home/student/catkin_ws/src/com2009_team20/snaps/")
        self.base_image_path.mkdir(parents=True, exist_ok=True) 
        self.pic_taken = False

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {self.node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            self.cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        print("---------------------------------------------------------------------------------------------------")
        height, width, _ = self.cv_img.shape
        crop_width = width - 800
        # Set to 100 so it doesn't detect pillars over walls or
        # colours on the floor
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

        if self.lower_hsv == (50, 70, 100) and self.upper_hsv == (70, 255, 255):
            self.colour_faced = "green"
        else:
            print("Invalid Colour")
        
        print(f"The Color Now Faced Is {self.colour_faced}")

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def scan_callback(self,scan_data):

        # Front Left
        left_view = scan_data.ranges[40:70]
        left_array = np.array(left_view)
        self.minD_left = left_array.min()
        self.minA_left = left_array.argmin()

        # Front Right
        right_view = scan_data.ranges[290:320]
        right_array = np.array(right_view)
        self.minD_right = right_array.min()
        self.minA_right = right_array.argmin()

        #Directly in front
        left_arc = scan_data.ranges[0:31]
        right_arc = scan_data.ranges[-30:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.minD_front = front_arc.min()
        maxD_front = front_arc.max()

        if self.m00 > self.m00_min:
            if (self.pic_taken == False):
                print(self.pic_taken)
                self.pic_taken = True
                show_and_save_image(self.cv_img, self.base_image_path)
        
        # if (self.minD_front < 0.25):
        #     self.turn = "noCrash"
        # # Going to hit right wall
        # elif (self.minD_right < 0.3):
        #     self.turn = "Left"
        # # Going to hit left wall
        # elif (self.minD_left < 0.3):
        #     self.turn = "Right"
        # # Space to go left, left traversal so follows 
        # elif (self.minD_left > 0.35):
        #     self.turn = "Left"
        # # Cant go left and cant go forward
        # elif (self.minD_front < 0.3):
        #     self.turn = "Right"
        # # No immediate issue so keeps going forward
        # else:
        #     self.turn = "No"

        # self.minInfo = f"Front: '{self.minD_front:.2f}', Left: '{self.minD_left:.2f}', Right: '{self.minD_right:.2f}'"

    
    def main_loop(self):
        current_yaw = 0.0
        current_displacement = 0.0
        self.target = ""
        while not self.ctrl_c:
            # here is where your code would go to control the motion of your
            # robot. Add code here to make your robot move in a square of
            # dimensions 1 x 1m...
            
            if self.turn:
                current_yaw = current_yaw + abs(self.theta_z - self.theta_z0)
                self.theta_z0 = self.theta_z
                if current_yaw >= 2*pi:
                    self.vel = Twist()
                    self.turn = False
                    current_yaw = 0.0
                    self.x0 = self.x
                    self.y0 = self.y
                    print(f"SEARCH INITIATED: The target beacon colour is {self.colour_arg[0]}")
                    self.target = self.colour_arg[0]
                    self.vel.angular.z = 0.0


                else: 
                    self.vel.angular.z = 0.3   

            else:
                self.vel.angular.z = 0.0
                self.vel.linear.x = 0.0

            # else:
            #     # move 1 m
            #     current_displacement = current_displacement + sqrt(pow(self.x-self.x0, 2) + pow(self.y - self.y0, 2))
            #     self.x0 = self.x
            #     self.y0 = self.y
            #     if current_displacement >= 1:
            #         self.vel = Twist()
            #         self.turn = True
            #         current_displacement = 0.0
            #         self.theta_z0 = self.theta_z
            #     else:
            #         self.vel.linear.x = 0.1 
 
            


            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            # maintain the loop rate @ 10 hz 
            self.rate.sleep()

def show_and_save_image(img, base_image_path): 
    full_image_path = base_image_path.joinpath("/the_beacon.jpg")

    print("Opening the image in a new window...")
    cv2.imshow("the_beacon", img) 
    print(f"Saving the image to '{full_image_path}'...")
    cv2.imwrite(str(full_image_path), img) 
    print(f"Saved an image to '{full_image_path}'\n"
        f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        f"file size = {full_image_path.stat().st_size} bytes") 
    print("Please close down the image pop-up window to continue...")
    cv2.waitKey(0) 

if __name__ == "__main__":
    node = Task4()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
