#!/usr/bin/env python3
import argparse
from turtle import right
import rospy
import numpy as np
import math
from pathlib import Path 
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
        self.map_path = Path.home().joinpath("catkin_ws/src/com2009_team20/maps/task5_map")
        # How node takes arguments for which colour to look for
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{self.node_name}' node.")
        cli.add_argument("target_colour", metavar="COL", default="Black",help="The name of a colour(Blue/Red/Yellow?Green)"
        )

        args, self.colour_arg = cli.parse_known_args(rospy.myargv()[1:])
        rospy.init_node('task5',anonymous=True)

        # Subscriber node for taking picture of beacon
        self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        
        # Subscriber node for getting scan data and odom data
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Publisher node for movement
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.vel = Twist()

        # Default values set incase colour_arg isn't right 
        # (Hue, Sat, Val)
        self.lower_hsv = (0,0,100)
        self.upper_hsv = (175,255,255)
        rospy.on_shutdown(self.shutdownhook)

        #These work for simulation need to check on real robots
        if self.colour_arg[0] == "blue":
            self.lower_hsv = (105, 50, 100)
            self.upper_hsv = (125, 100, 255)
        elif self.colour_arg[0] == "red":
            self.lower_hsv = (150, 50, 100)
            self.upper_hsv = (175, 225, 255)
        elif self.colour_arg[0] == "yellow":
            self.lower_hsv = (10,50,100)
            self.upper_hsv = (20,255,255)
        elif self.colour_arg[0] == "green":
            self.lower_hsv = (87, 100, 100)
            self.upper_hsv = (100, 255, 255)
        else:
            print("Invalid Colour Choice")
        print(f"TASK 5 BEACON: The target is {self.colour_arg[0]}")

        # Initialises class variables
        self.turn = "No"
        self.minD_left = 0 
        self.minD_right = 0 
        self.minD_front = 0 
        self.maxD_front = 0
        self.ctrl_c = False
        self.counter = 0
        self.m00 = 0
        self.m00_min = 10000
        self.spin_counter = 0
        self.base_image_path = Path.home().joinpath("catkin_ws/src/com2009_team20/snaps/")
        print(self.base_image_path)
        self.base_image_path.mkdir(parents=True, exist_ok=True) 
        self.pic_taken = False
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        

    def shutdownhook(self):
        self.pub.publish(Twist())
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            self.cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            print("____________________________________________________________________")
        
        height, width, _ = self.cv_img.shape
        crop_width = width -48

        # Set to 100 so it doesn't detect pillars over walls or
        # colours on the floor
        crop_height = 80
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = self.cv_img[crop_y+20:crop_y+crop_height+20, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img,self.lower_hsv, self.upper_hsv)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def scan_callback(self,scan_data):

        # Re-used movement code from Task 3 ----  
         # Front left side view
        left_view = scan_data.ranges[30:60]
        left_array = np.array(left_view)
        self.minD_left = left_array.min()

        self.minA_left = left_array.argmin()

        # Front right side view
        right_view = scan_data.ranges[300:330]
        right_array = np.array(right_view)
        self.minD_right = right_array.min()

        # Front 30 degree view, for not hitting walls
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.minD_front = front_arc.min()

        # For seeing if a U-Turn is needed 
        wide_front_left = scan_data.ranges[0:91]
        wide_front_right = scan_data.ranges[-90:]
        wide_front_arc = np.array(wide_front_left[::-1] + wide_front_right[::-1])
        self.maxD_front = wide_front_arc.max()

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
        #remove before submit
        self.counter += 1



    def main_loop(self):
        rate = rospy.Rate(30)
        while not self.ctrl_c:
            # How often updates + saves map, also does a spin to look for stuff
            if self.counter > 300:
                print(f"Saving map at time: {rospy.get_time()}...")
                node = roslaunch.core.Node(package="map_server",
                           node_type="map_saver",
                           args=f"-f {self.map_path}")
                self.launch.launch(node)
                self.counter = 0 

            self.vel.linear.x = 0.13
            if self.m00 > self.m00_min:
                print(self.m00)
                print("--------------------------------------------------------------------")
                #Make entire width in frame 
                if self.pic_taken == False:
                    self.vel.linear.x = 0 
                    self.vel.angular.z = 0
                    self.pub.publish(self.vel)
                    while (self.pic_taken == False):
                        print("_______________TAking photo__________________")
                        print(self.cy)
                        if self.cy > 450 and self.cy < 650:
                            print(self.pic_taken)
                            self.pic_taken = True
                            show_and_save_image(self.cv_img, self.base_image_path)
                        else : 
                            if self.cy < 450:
                                self.vel.angular.z = 0.5
                            elif self.cy > 650 :
                                self.vel.angular.z = -0.5
                            self.pub.publish(self.vel)
    
            # Reused from task 3 -----
            # Room to move left so follow it 
            if self.counter > 30:
                print(f"min_distF{self.minD_front:.5f}, min_distR{self.minD_right:.5f}, min_distL{self.minD_left:.5f}")
    
            if (self.minD_left == 0):
                self.minD_left = 10
            if (self.minD_right == 0):
                self.minD_right = 10
            if (self.minD_front == 0):
                self.minD_front = 10

            if (self.minD_left > 0.4) and (self.minD_front > 0.5):
                self.turn = "left"
                self.vel.linear.x = 0.08
                self.vel.angular.z = 0.3
            # Can't go left, but can follow left wall forward
            elif (self.minD_front > 0.5) and (self.minD_left > 0.3) and (self.minD_left < 0.4):
                self.turn = "forward"
                self.vel.linear.x = 0.08
                self.vel.angular.z = 0.1
            # Can't go left or forward, but can go right
            elif (self.minD_left < 0.3) and (self.minD_front > 0.5):
                self.turn = "right"
                self.vel.linear.x = 0.08
                self.vel.angular.z = -0.3
            # Nowhere in front to go, does a 180 turn 
            elif (self.maxD_front < 0.65):
                self.turn = "uTurn"
                self.vel.linear.x = 0.018
                self.vel.angular.z = math.pi/2
                self.pub.publish(self.vel)
                rospy.sleep(2)
            # If not, must be wall directly ahead
            else:
                self.turn = "Stop, wall ahead"
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
                #Spins in the direction with most space to move
                self.vel.angular.z = -0.7
            self.pub.publish(self.vel) 
            if self.counter > 30:   
                print(self.turn)
                print(f"min_distF{self.minD_front:.5f}, min_distR{self.minD_right:.5f}, min_distL{self.minD_left:.5f}")
                self.counter = 0
        rate.sleep()    

def show_and_save_image(img, base_image_path): 
    full_image_path = base_image_path.joinpath("the_beacon.jpg")
    print(full_image_path)
    #print("Opening the image in a new window...")
    #cv2.imshow("the_beacon", img) 
    print(f"Saving the image to '{full_image_path}'...")
    cv2.imwrite(str(full_image_path), img) 
    print(f"Saved an image to '{full_image_path}'\n"
        f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        f"file size = {full_image_path.stat().st_size} bytes") 
    print("Please close down the image pop-up window to continue...")
    #cv2.waitKey(0) 
    print("continue")

if __name__ == "__main__":
    node = Task5()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass