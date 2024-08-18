#!/usr/bin/env python3 

import rospy 
from pid_control import PID_PositionControl 
from geometry_msgs.msg import Pose ,Twist 
from std_msgs.msg import Int16

import numpy as np 
import math as m 

distance_control = PID_PositionControl()
angle_control = PID_PositionControl()

object_position = None 
control_mode = 0 

# 0 offcontrol 
# 1 control only angle 
# 2 control distance and angle 

def object_pose_callback(msg) : 
    global object_position 
    object_position = np.array([msg.position.x,msg.position.y,msg.position.z])

def control_mode_callback(msg) : 
    global control_mode 
    control_mode = msg.data

if __name__ == "__main__" :
    rospy.init_node("turtlebot_drive")

    distance_gain = rospy.get_param("~distance_control/gain",[0.,0.,0.])
    distance_max_sum_error = rospy.get_param("~distance_control/max_sum_error",1000)
    distance_min_error = rospy.get_param("~distance_control/min_error",0.)
    distance_max_output = rospy.get_param("~distance_control/max_output",10.0)

    angle_gain = rospy.get_param("~angle_control/gain",[0.,0.,0.])
    angle_max_sum_error = rospy.get_param("~angle_control/max_sum_error",1000)
    angle_min_error = rospy.get_param("~angle_control/min_error",0.)
    angle_max_output = rospy.get_param("~angle_control/max_output",10.0)

    distance_control.set_gain(distance_gain[0],distance_gain[1],distance_gain[2])
    distance_control.set_max_sum_error(distance_max_sum_error)
    distance_control.set_min_error(distance_min_error)
    distance_control.set_max_output(distance_max_output)

    angle_control.set_gain(angle_gain[0],angle_gain[1],angle_gain[2])
    angle_control.set_max_sum_error(angle_max_sum_error)
    angle_control.set_min_error(angle_min_error)
    angle_control.set_max_output(angle_max_output)

    object_pose_sub = rospy.Subscriber("object_pose",Pose,object_pose_callback)
    control_mode_sub = rospy.Subscriber("control_mode",Int16,control_mode_callback)
    cmd_vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)

    rate = rospy.Rate(10)

    try : 
        while not rospy.is_shutdown() : 
            if object_position is not None :

                object_distance = np.sqrt(object_position[0]*object_position[0] + object_position[1]*object_position[1] + object_position[2]*object_position[2])
                object_direction = object_position / object_distance

                object_angle = m.atan2(object_direction[1],object_direction[0])

                if control_mode == 0 : 
                    linear_vel = 0 
                    angular_vel = 0 
                elif control_mode == 1 : 
                    linear_vel = 0 
                    angular_vel = angle_control.control(object_angle,0.)
                elif control_mode == 2 : 
                    linear_vel =  distance_control.control(object_distance,0.2)
                    angular_vel = angle_control.control(object_angle,0.)
                
                cmd_vel = Twist()
                cmd_vel.linear.x = linear_vel 
                cmd_vel.angular.z = angular_vel
                cmd_vel_pub.publish(cmd_vel)

            rate.sleep() 

    except rospy.ROSInterruptException : 
        cmd_vel_pub.publish(Twist())
        pass 