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
    
    x2 = pose_message.x
    y2 = pose_message.y
    theta2 = pose_message.theta
    
def follow():
    velocity_message = Twist()

    global x
    global y
    global z
    global theta
    
    global x2
    global y2
    global z2
    global theta2
    
    angle_to_dummy = math.atan2(y2 - y, x2 - x)
    relative_angle = angle_to_dummy - theta

    if relative_angle > math.pi:
        relative_angle -= 2 * math.pi
    elif relative_angle < -math.pi:
        relative_angle += 2 * math.pi
        
    theta_deg = math.degrees(relative_angle)
    
    angle = int(theta_deg)

    euc_dist = math.sqrt((x - x2)**2 + (y - y2)**2)

    reader = csv.reader(open(r"/home/rodrigo/Documents/ros_ws/Benito/src/benito2_2/src/velocidades_lineal_2.csv", "r"), delimiter=",")

    x1_list = list(reader)
    tabla_lineal = numpy.array(x1_list).astype("float")


    reader = csv.reader(open(r"/home/rodrigo/Documents/ros_ws/Benito/src/benito2_2/src/velocidades_angular_2.csv", "r"), delimiter=",")

    x2_list = list(reader)
    tabla_angular = numpy.array(x2_list).astype("float")
    
    velocity_message.linear.x = tabla_lineal[int(angle+179)][int(euc_dist-1)]*0.635
    velocity_message.angular.z = tabla_angular[int(angle+179)][int(euc_dist-1)] 
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
            follow()
            

    except rospy.ROSInterruptException:        
        pass
