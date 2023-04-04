#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from geometry_msgs.msg import Twist
import math
class Publisher(): 

    def __init__(self): 
        self.node_name = "task1_pub" 
        topic_name = "cmd_vel" 

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(1) 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

        self.counter = 0


    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True
              

    def main_loop(self):
        #init_time = rospy.get_rostime().secs
        start_time = rospy.get_rostime()
        #anti-clockwise circle
        vel = Twist()
        move_time = (2 * math.pi) / 0.23
        if (rospy.get_rostime().secs - start_time.secs) <= (move_time):
            vel.linear.x = 0.115 # m/s
            vel.angular.z = 0.23 # rad/s
            self.pub.publish(vel)
            
        elif move_time < (rospy.get_rostime().secs - start_time.secs) <= (2 * move_time):
            vel.linear.x = 0.115
            vel.angular.z = -0.23
            self.pub.publish(vel)
            
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.pub.publish(vel)
            
            #while (rospy.get_rostime().secs - StartTime.secs) < move_time: 
            #    continue

            #rospy.loginfo('duration seconds have elapsed, stopping the robot...')

            #clockwise circle
            #initial_time = rospy.get_rostime()
            #while (rospy.get_rostime().secs - initial_time.secs) < move_time: 
            #    continue

            


        

            #stop
            #vel.linear.x = 0.0
            #vel.angular.z = 0.0
            #self.pub.publish(vel)
            #rospy.loginfo('all elapse, stopping the robot...')
    
if __name__ == '__main__': 
    publisher_instance = Publisher()
    #start_time = rospy.get_rostime().secs
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass