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
vel2 = 0

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
def poseCallback2(pose_message):
    global x2
    global y2
    global z2
    global theta2
    
    x2 = pose_message.x
    y2 = pose_message.y
    theta2 = pose_message.theta

def velCallback2(vel_message2):
    global vel2
    
    vel2 = vel_message2.linear.x

def orientate (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        ka = 2.5
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

    global x2
    global y2
    global theta2
    global vel2

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        			
        distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        kv = 0.5
        linear_speed = kv * distance

        ka = 4.5
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        nuevoangulo= desired_angle_goal-theta 
        if nuevoangulo > math.pi:
            nuevoangulo -= 2*math.pi
        elif nuevoangulo < -math.pi:
            nuevoangulo += 2*math.pi 
       
        dtheta = nuevoangulo
        angular_speed = ka * (dtheta)
            
        dy = np.abs(y2 - y)


        #if dy < 1.0:
        #    if vel2 < 0.5 and vel2 >= 0.0:
        #        linear_speed = kv*(vel2)
        if dy < 2.0:# and dy >= 1.0:}
            ka += 0.5
            kv = 0.7
            if desired_angle_goal < 3*math.pi/4 and desired_angle_goal >= 0:
                desired_angle_goal = 2.5
            else:
                desired_angle_goal = 5.7
            nuevoangulo= desired_angle_goal-theta 
            if nuevoangulo > math.pi:
                nuevoangulo -= 2*math.pi
            elif nuevoangulo < -math.pi:
                nuevoangulo += 2*math.pi 
            if np.abs(x2 - xgoal) <= 1.0:
                linear_speed = kv*vel2 + 0.1
            else:
                linear_speed = 0.7*distance + (vel2*0.75)
            angular_speed = 7.0*nuevoangulo
            
        else: 
            kv = 0.5
            angular_speed = ka * (dtheta)
        linear_speed = kv * distance

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (distance < 0.01):
            #if desired_angle_goal < math.pi and desired_angle_goal >= 0:
            orientate(5.0, 11.0)
            #else:
            #    orientate(5.0, 0.5)

            break

if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        
        position_topic_dummy = "/turtle2/pose"
        pose_subscriber_dummy = rospy.Subscriber(position_topic_dummy, Pose, poseCallback2)

        cmd_vel_topic2 = "/turtle2/cmd_vel"
        velocity_subscriber2 = rospy.Subscriber(cmd_vel_topic2, Twist, velCallback2)

        time.sleep(2)     

	
        while(True):
            time.sleep(2.0)
            orientate(5.0, 10)
            time.sleep(2.0)
            go_to_goal(5.0, 10)	 

            time.sleep(2.0)
            orientate(5.0, 1)
            time.sleep(2.0)
            go_to_goal(5.0, 1) 



    except rospy.ROSInterruptException:        
	    pass