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

        # NOT SURE IF NEEDED ---------
        #direct_left_view = scan_data.ranges[65:115]
        #direct_left_array = np.array(direct_left_view)
        #self.minD_direct_left = direct_left_array.min()

        #back_view = scan_data.ranges[160:200]
        #back_arc = np.array(back_view)
        #self.minD_back = back_arc.min()

        
                #   Front Dist < 0.3 = Stop Moving

                #if (self.minD_left < 0.25):
                    #self.vel.angular.z = -1.8 + (self.minA_left * 0.04)

                    #if (self.minD_left < 0.15 or self.minD_front < 0.15):
                        #self.vel.linear.x = 0.00
                    #else : 
                        #self.vel.linear.x = (self.minD_left * 0.8)
                #else:
                    #if self.minD_front < 0.6:
                        #self.vel.angular.z = -0.5 - ((0.6-self.minD_front)* (10/3) )
                        #self.vel.linear.x = 0.2 - ((0.6 - self.minD_front)*0.4)
                    #else: 
                        #self.vel.angular.z = -1.4
                    #20% corner, goes straight into wall as it doesn't turn as fast
                    #calculate angle by how close wall in front is
                    # 0.3 - 0.6, 0.3 = -1.5 , 0.6 = -0.5
                    # closer = faster turn,
                    #self.vel.linear.x = 0.20
                    #if (self.minD_right > 0.44 and self.minD_right < 0.6):
                        #self.vel.angular.z = -0.5
                    #else :
                        #self.vel.angular.z = -1.5
                    #if self.minD_left > 0.25:
                        #self.vel.angular.z = -1.2
                    #elif self.minD_right < 0.6:
                        #self.vel.angular.z = self.minD_right * -3
                   # else:
                        #self.vel.angular.z = -1.2
        # -------

        # Front left side view
        left_view = scan_data.ranges[40:80]
        left_array = np.array(left_view)
        # Gets distance (m) that is the closest
        self.minD_left = left_array.min()
        # Gets angle (degree) attached to the closest distance
        # LEFT SIDE: 0 = In front of bot , 40 = side of bot
        self.minA_left = left_array.argmin()

        right_view = scan_data.ranges[280:320]
        right_array = np.array(right_view)
        # Gets distance (m) that is the closest on the right
        self.minD_right = right_array.min()
        # Gets angle (degree) that is the attached to the closest distance
        # RIGHT SIDE: 40 = In front of bot , 0 = side of bot
        self.minA_right = right_array.argmin()
        
        # View of directly in front of bot 
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.minD_front = front_arc.min()
        maxD_front = front_arc.max()

        # MAYBE ADD as failsafe, get rid if it affects too much normal moving
        #if (self.minD_front < 0.25 or self.minD_full_front < 0.15):
            #self.turn = "noCrash"

        # Distance between left and right wall
        # +ve = Closer to right wall , -ve = Closer to left
        self.left_right_dist = self.minD_left - self.minD_right
        if (self.minD_left < 0.5 and self.minD_right < 0.5 and maxD_front < 0.55):
            self.turn = "uTurn"
        elif (self.left_right_dist > 0.1):
            self.turn = "Left"
        elif (self.minD_front < 0.3):
            self.turn = "stopLeft"
        elif (self.left_right_dist < -0.1):
            self.turn = "Right"
        elif (self.minD_left > 0.45):
            self.turn = "Left"
        elif (maxD_front < 0.65):
            self.turn = "uTurn"
        else:
            self.turn = "No"

        #Prints a lot of data
        self.minInfo = f"Front: '{self.minD_front:.2f}',  Left: '{self.minD_left:.2f}', Right: '{self.minD_right:.2f}', MaxFront: '{maxD_front:.2f}', leftMinA: '{self.minA_left:.2f}',rightMinA: '{self.minA_right:.2f}'"

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
        if self.counter > 25:
            self.counter = 0
            print(f"'{self.minInfo}'")
            print(f"he is turning'{self.turn}'")
        else:
            self.counter += 1

    def main_loop(self):
        rate = rospy.Rate(40)
        while not self.ctrl_c:
            if self.turn == "Left":
                # IF TOO CLOSE TO THE RIGHT WALL
                if (self.minD_right < 0.25):
                    #Turning Speed 
                    # LB 0.2 : Closest angle is directly too right
                    # UB 1.8 : Wall close to in front 
                    self.vel.angular.z = 0.1 + (self.minA_right * 0.04)

                    #Linear Speed
                    # If very close to right, don't move
                    if (self.minD_right < 0.15):
                        self.vel.linear.x = 0.00
                    
                    # Otherwise, scale (0.15m to 0.25m)
                    else : 
                        # 0.25m = 0.2m/s
                        # 0.15m = 0.12m/s
                        #self.vel.linear.x = (self.minD_right * 0.8)
                        # 0.25m = 0.15m/s
                        # 0.15m = 0.05m/s
                        self.vel.linear.x = 0.05 +((0.25 - self.minD_right))
                    
                # IF TURNING BECAUSE TOO FAR FROM LEFT WALL
                else:

                    #If closer than 0.6m away from wall in front, scale turning and linear speed
                    if self.minD_front < 0.6:
                        self.vel.angular.z = 0.35 + ((0.6-self.minD_front)* (10/3) )
                        self.vel.linear.x = 0.25 - ((0.6 - self.minD_front)*0.4)
                        #0.1 = - 0.2
                        #0.6 = - 0 

                    #Further than 0.6m away from front wall
                    else:
                        self.vel.linear.x = 0.20
                        self.vel.angular.z = 1.4


            elif self.turn == "stopLeft":
                #When linear speed is -ve, angular needs to be the negation of whichever way
                #you actually wanna go
                self.vel.angular.z = -1
                self.vel.linear.x = -0.2

            elif self.turn == "Right" or self.turn =="Right2":
                #HOW FAST TURN based on CLOSEST ANGLE
                #   Angle = closer to front = turn faster (1.8)
                #         = closer to side  = turn slower (0.3)
                 
                #HOW FAST MOVE based on CLOSEST METER
                #   Distance = lower (<0.25) = move slower (0.05)
                #            = higher (> 0.7) = move faster (0.25)
                # if self.minA_left <
                
                self.vel.angular.z = -1.7 + (self.minA_left * 0.04)
                if self.minD_front < 0.6:
                    self.vel.linear.x = 0.18 - ((0.6 - self.minD_front)*0.4)
                else: 
                    self.vel.linear.x = 0.25
            
            elif self.turn == "noCrash":
                front_back_range = self.minD_front - self.minD_back
                while (front_back_range > 0.05 or front_back_range < -0.05):
                    front_back_range = self.minD_front - self.minD_back
                    if front_back_range > 0:
                        self.vel.linear.x = 0.1
                    else : 
                        self.vel.linear.x = -0.1
                    self.vel.angular.z = 0
                    self.pub.publish(self.vel)          
                #move him between the area in front and behind him
                #while loop until the middle then continue
                rospy.sleep(1)

            elif self.turn == "uTurn":
                self.vel.linear.x = 0.018
                self.vel.angular.z = math.pi/2
                self.pub.publish(self.vel)
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
