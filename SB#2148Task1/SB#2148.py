#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
import math
from math import atan2
from math import sin


roll = pitch = yaw = 0.0
P = 1.4         #Proportionality constant
x = 0.0         #Current position(Y) of vehicle
y = 0.0         #Current position(X) of vehicle
count = 0.0     # This user defined variable used in obstacle function

global goal
goal=Point()

#Laser_callback is taking input from laser sensor

def laser_callback(msg):
    global regions
    regions = {
        'bright':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'bleft':   min(min(msg.ranges[576:713]), 10),
    }

#Waypoints function is setting the next waypoint for the vehicle to move to.

def Waypoints(n):
    global goal_x
    global goal_y
    goal_x=[0.65,1.23,1.78,2.46,3.14,3.78,5.00,5.62,6.00,12.50]
    goal_y=(2*(math.sin(goal_x[n])))*(math.sin((goal_x[n]/2)))
    goal.x = round(goal.x,2)
    goal.y = round(goal.y,2)

    
#odom_callback simply stores and gives the current position of the vehicle and theta.
 
def odom_callback(data):
    global x
    global y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global roll, pitch, yaw
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

#This help the vehicle to navigate through the obstacle.

def obstacle():
    global count
    if(regions['front']<2):
        velocity_msg.linear.x = 0.0
        pub.publish(velocity_msg)
    
    while x<11.5:
        if regions['front'] < 1.5 and regions['fleft'] < 1.5 and regions['fright'] < 1.5:
            velocity_msg.angular.z = 0.8
            velocity_msg.linear.x = 0.0
            count = count + 1
            rospy.loginfo_once(regions)
            pub.publish(velocity_msg)
        elif regions['front'] > 1.5 and regions['fleft'] > 2.0 and regions['fright'] < 1.5:
            velocity_msg.linear.x = 0.4
            velocity_msg.angular.z = 0.0
            rospy.loginfo_once(regions)
            pub.publish(velocity_msg)
        elif regions['front'] > 1.5 and regions['fleft'] > 1.5 and regions['fright'] > 1.5:
            if(count==0):
                velocity_msg.angular.z = 0.0
                velocity_msg.linear.x = 0.3
                rospy.loginfo_once(regions)
                pub.publish(velocity_msg)
            else:
                velocity_msg.angular.z = -0.6
                velocity_msg.linear.x = 0.2
                pub.publish(velocity_msg)
        else:
            pass





rospy.init_node('ebot_controller')                                  #This is a very important line as it tells rospy the name of your node.

sub = rospy.Subscriber ('/odom', Odometry, odom_callback)           #These lines declare that the node subscribes to the /odom topic
sub1 = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)     #These lines declare that the node subscribes to the /ebot/laser/scan
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)              #declares that your node is publishing to the cmd_vel topic.

velocity_msg= Twist()                                               #This is a very important line as it provides linear and angular velocity vectors of vehicle.
rate = rospy.Rate(4)                                                #This line creates a Rate object rate.

#move function is moving the vehicle to the point provided by waypoint function.

def move(pts):
    Waypoints(pts)
    goal.x=goal_x[pts]
    goal.y = goal_y
    inc_x = goal.x - x
    inc_y = goal.y - y
    target_angle = atan2(inc_y,inc_x)
    velocity_msg.angular.z = P  * (target_angle - yaw)
    velocity_msg.linear.x = 0.2

#This control_loop() is main loop which is moving the vehicle in gazebo.

def control_loop():
    
    while not rospy.is_shutdown(): 
       move(0)
       if x >= goal.x:
           move(1)
       if x >= goal.x:
           move(2)
       if x >= goal.x:
           move(3)
       if x >= goal.x:
           move(4)
       if x >= goal.x:
           move(5)
       if x >= goal.x:
           move(6)
       if x >= goal.x:
           move(7)
       if x >= goal.x:
           Waypoints(8)
           goal.x=goal_x[8]
           goal.y = goal_y
           inc_x = goal.x - x
           inc_y = goal.y - y
           target_angle = atan2(inc_y,inc_x)
           velocity_msg.angular.z = 0.9  * (target_angle - yaw)
           velocity_msg.linear.x = 0.2
       if x >= goal.x:
           Waypoints(9)
           goal.x=goal_x[9]
           goal.y = goal_y
           inc_x = goal.x - x
           inc_y = goal.y - y
           target_angle = atan2(inc_y,inc_x)
           velocity_msg.angular.z = 0.9  * (target_angle - yaw)
           velocity_msg.linear.x = 0.2
       if x >= 8.0: 
           velocity_msg.linear.x=0.0
           obstacle() 
       if x >= 11.5:
           Waypoints(9)
           goal.x=goal_x[9]
           goal.y = goal_y
           inc_x = goal.x - x
           inc_y = goal.y - y
           target_angle = atan2(inc_y,inc_x)
           velocity_msg.angular.z = 0.9  * (target_angle - yaw)
           velocity_msg.linear.x = 0.2
           if x >= goal.x:
                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = 0.0 
       pub.publish(velocity_msg)
       print("Controller message pushed at {}".format(rospy.get_time()))
       rate.sleep() 
                
  

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

    






          
 

                            



                
       
  



