#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
import sys

def move_turtle(lin_vel,ang_vel):

    rospy.init_node('node_turtle_revolve', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose', Pose, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    circle_rad= lin_vel/ang_vel #radius of circle
    distance = 2.2*3.142857*circle_rad #circumference of circle(distance to travel by turtle)
    vel = Twist()
    while not rospy.is_shutdown():
	#Setting the current time for distance calculus
        t0=time.time() #time0 (to calculate current distance)
	current_distance = 0 #distance travelled by turtle
	while not(current_distance>distance):
	    vel.linear.x = circle_rad*ang_vel #linear vel = radius*angular velocity
            vel.angular.z = lin_vel/circle_rad #angular velocity = lin vel/rad
	    pub.publish(vel)
            rospy.loginfo("Moving in a circle\n%f",current_distance)
            t1=time.time() #time1 (to calculate current distance)
	    current_distance= lin_vel*(t1-t0)#distance travelled by turtle
        vel.linear.x =0  #to stop robot when it reaches goal
        pub.publish(vel) #publishing velocity message to the bot to stop
        rospy.loginfo("Goal reached")
        break
	
        
            
              
if __name__ == '__main__':
    try:
        move_turtle(1,1)
    except rospy.ROSInterruptException:
        pass
        
       
