#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import csv
import numpy 

x = 0
y = 0
z = 0
theta = 0

x2 = 0
y2 = 0
z2 = 0
theta2 = 0
linear_vel = 0.0
angular_vel = 0.0

def poseCallback(pose_message):
    global x
    global y
    global z
    global theta
    
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
    
def poseCallback2(pose_message):
    global x2
    global y2
    global z2
    global theta2
    global linear_vel
    global angular_vel
    
    x2 = pose_message.x
    y2 = pose_message.y
    theta2 = pose_message.theta
    linear_vel = pose_message.linear_velocity

def look_at():
    global linear_vel
    velocity_message = Twist()

    global x
    global y
    global z
    global theta
    
    global x2
    global y2
    global z2
    global theta2

    
    lineal = linear_vel/3

    
    angle_to_dummy = math.atan2(y2 - y, x2 - x)
    relative_angle = angle_to_dummy - theta

    if relative_angle > math.pi:
        relative_angle -= 2 * math.pi
    elif relative_angle < -math.pi:
        relative_angle += 2 * math.pi
        
    angle = math.degrees(relative_angle)
    reader = csv.reader(open(r"/home/rodrigo/Documents/ros_ws/Benito/src/benito2_1/src/vel_ang2.csv", "r"), delimiter=",")
    x_list = list(reader)
    table = numpy.array(x_list).astype("float")
    
    velocity_message.linear.x = 0.0
    

    velocity_message.angular.z = table[int(angle+179)][int(lineal-1)]
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        cmd_vel_topic = '/turtle1/cmd_vel'
        
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        position_topic = "/turtle1/pose"
        position_topic2 = "/turtle2/pose"
        
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        pose_subscriber = rospy.Subscriber(position_topic2, Pose, poseCallback2)
        
        time.sleep(2)    
 
        
        while(True):
            look_at()


    except rospy.ROSInterruptException:        
        pass