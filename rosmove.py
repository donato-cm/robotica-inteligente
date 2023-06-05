#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import numpy as np

#t1
x = 0
y = 0
z = 0
theta = 0

#t2
x2 = 0
y2 = 0
z2 = 0
theta2 = 0

#t1
def poseCallback(pose_message):
    global x
    global y
    global z
    global theta
    
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

#t2
def poseCallback2(pose_message2):
    global x2
    global y2
    global z2
    global theta2
    
    x2 = pose_message2.x
    y2 = pose_message2.y
    theta2 = pose_message2.theta


def orientate (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        ka = 1.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        nuevoangulo= desired_angle_goal-theta 
        if nuevoangulo > math.pi:
            nuevoangulo -= 2*math.pi
        elif nuevoangulo < -math.pi:
            nuevoangulo += 2*math.pi 
       
        dtheta = nuevoangulo
       
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = 0.0
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (dtheta < 0.01):
            break

def go_to_goal (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        #kv = 0.5			
        """
        if (theta < -3*np.pi/4 and theta > -np.pi/4):
            if (y > y2):
                xgoal = (x3 - x2)/2
                ygoal = y2
            elif y == y2:
                xgoal = x_bueno
                ygoal = y_bueno
        elif (theta < 3*np.pi/4 and theta > np.pi/4):
            if (y < y2):
                xgoal = (x3 - x2)/2
                ygoal = y2
            elif (y==y2):
                xgoal = x_bueno
                ygoal = y_bueno
        """

        distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        dx = x-x2
        dy = y-y2
        euc_dist = np.sqrt(dx**2 + dy**2)
        #euc_dist = np.sqrt((x-x2)**2 + (y-y2)**2)
        
        if ((theta < -3*np.pi/4 and theta > -np.pi/4) or (theta < 5*np.pi/3 and theta > 4*np.pi/3)):
            if dy>=0.0: # and np.abs(euc_dist) < 2.0:
                if (np.abs(euc_dist) <= 1.0):
                    kv = -0.7
                elif(np.abs(euc_dist) > 1.0 and np.abs(euc_dist) <= 1.5):
                    kv = 0
                else:
                    kv = 0.5
            elif dy<0:
                kv = 1.0

        elif (theta < 3*np.pi/4 and theta > np.pi/4):
            if dy<=0.0: # and np.abs(euc_dist) < 2.0:
                if (np.abs(euc_dist) <= 1.0):
                    kv = -0.7
                elif(np.abs(euc_dist) > 1.0 and np.abs(euc_dist) <= 1.5):
                    kv = 0
                else:
                    kv = 0.5
            elif dy>0:
                kv = 1.0
        
        else:
            kv = 0.5
        """
        if (np.abs(euc_dist) <= 1):
                kv = -0.2
        elif(np.abs(euc_dist) > 1 and np.abs(euc_dist) <= 1.5):
            kv = 0
        else:
            kv = 0.5
        """
        

        linear_speed = kv * distance #############################################

        ka = 1.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        nuevoangulo= desired_angle_goal-theta 
        if nuevoangulo > math.pi:
            nuevoangulo -= 2*math.pi
        elif nuevoangulo < -math.pi:
            nuevoangulo += 2*math.pi 
       
        dtheta = nuevoangulo
       
            
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (distance < 0.01):
            break

if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        #t1
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)     

        #t2
        cmd_vel_topic2 = '/turtle2/cmd_vel'
        velocity_publisher2 = rospy.Publisher(cmd_vel_topic2, Twist, queue_size = 10)

        position_topic2 = "/turtle2/pose"
        pose_subscriber2 = rospy.Subscriber(position_topic2, Pose, poseCallback2)
        time.sleep(2)     
	
        while(True):
            
            time.sleep(1.0)
            orientate(5.0, 2.0)
            time.sleep(1.0)
            go_to_goal(5.0, 2.0)

            time.sleep(1.0)
            orientate(5.0, 8.0)
            time.sleep(1.0)
            go_to_goal(5.0, 8.0)

    except rospy.ROSInterruptException:        
        pass
