#!/usr/bin/env python3
import argparse
from turtle import right
import rospy
import numpy as np
import math
import roslaunch

# For creating images
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class Task5:
    def __init__(self):
        self.node_name ="task5"

        # How node takes arguments for which colour to look for
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{self.node_name}' node.")
        cli.add_argument("colour", metavar="COL", default="Black",help="The name of a colour(Blue/Red/Yellow?Green)"
        )

        args, self.colour_arg = cli.parse_known_args(rospy.myargv()[1:])

        rospy.init_node('task5',anonymous=True)

        # Subscriber node for taking picture of beacon
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        
        # Subscriber node for getting scan data and odom data
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Publisher node for movement
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()

        # Initialises class variables
        self.turn = "No"
        self.ctrl_c = False
        self.counter = 0
        self.m00 = 0
        self.m00_min = 10000

        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.pub.publish(Twist())
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        # Set to 100 so it doesn't detect pillars over walls or
        # colours on the floor
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Default values set incase colour_arg isn't right 
        # (Hue, Sat, Val)
        lower_hsv = (0,0,100)
        upper_hsv = (175,255,255)

        #These work for simulation need to check on real robots
        if self.colour_arg == "Blue":
            lower_hsv = (115, 50, 100)
            upper_hsv = (160, 255, 255)
        elif self.colour_arg == "Red":
            lower_hsv = (0, 50, 100)
            upper_hsv = (10, 255, 255)
        elif self.colour_arg == "Yellow":
            lower_hsv = (25,50,100)
            upper_hsv = (40,255,255)
        elif self.colour_arg == "Green":
            lower_hsv = (50, 70, 100)
            upper_hsv = (70, 255, 255)
        else:
            print("invalid colour given")

        mask = cv2.inRange(hsv_img,lower_hsv, upper_hsv)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

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

        #If blob is detected
        # Enter a while loop 
        #   Move towards it until it fills a certain height/width
        #   Follow normal movement rules, avoiding walls etc
        #   
        if self.m00 > self.m00_min:
            print("Blob Detected")

        # Going to hit front wall
        if (self.minD_front < 0.25):
            self.turn = "noCrash"
        # Going to hit right wall
        elif (self.minD_right < 0.3):
            self.turn = "Left"
        # Going to hit left wall
        elif (self.minD_left < 0.3):
            self.turn = "Right"
        # Space to go left, left traversal so follows 
        elif (self.minD_left > 0.35):
            self.turn = "Left"
        # Cant go left and cant go forward
        elif (self.minD_front < 0.3):
            self.turn = "Right"
        # No immediate issue so keeps going forward
        else:
            self.turn = "No"

        self.minInfo = f"Front: '{self.minD_front:.2f}', Left: '{self.minD_left:.2f}', Right: '{self.minD_right:.2f}'"

    def odometry_callback(self,odom):
        pose = odom.pose.pose
        orientation = pose.orientation

        curr_x = pose.position.x
        curr_y = pose.position.y
        curr_z = pose.position.z

        (roll, pitch, yaw) = euler_from_quaternion([curr_x, 
                     curr_y, curr_z, orientation.w], 
                     'sxyz')

        if self.counter > 25:
            self.counter = 0
            print(f"'{self.minInfo}'")
            print(f"he is turning'{self.turn}'")
        else:
            self.counter += 1

    def main_loop(self):
        print(self.colour_arg)
        rate = rospy.Rate(40)
        while not self.ctrl_c:
            self.vel.linear.x = 0.13
            
            if self.turn == "noCrash":
                self.vel.linear.x = -0.1
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
                rospy.sleep(1)
            elif self.turn == "Left":
                self.vel.angular.z = 0.8
            elif self.turn == "Right":
                self.vel.angular.z = -0.8
            else:
                self.vel.angular.z = 0 
                self.vel.linear.x = 0.15
            self.pub.publish(self.vel)
            rate.sleep()
        
            map_path = "~/catkin_ws/src/com2009_team20/maps/task5_map"

            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()

            print(f"Saving map at time: {rospy.get_time()}...")
            node = roslaunch.core.Node(package="map_server",
                                    node_type="map_saver",
                                    args=f"-f {map_path}")
            process = launch.launch(node)

if __name__ == "__main__":
    node = Task5()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass