#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import subprocess
import os
import time
import math 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid , Odometry , Path

kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05

class GotoPoint():
    def __init__(self):
        self.goal_x_list = []
        self.goal_y_list = []
        self.position_x_list = []
        self.position_y_list = []
        self.error_list = []
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        path=rospy.wait_for_message('/path_rrt',Path)
        # for data in path.poses:
        #     print("way point:", data.pose.position.x, data.pose.position.y)
        self.cmd_vel = rospy.Publisher('/tbot/cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        stop_cmd=Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")
        prev_x=1
        prev_y=1

        print(len(path.poses))
        for c,data in enumerate(path.poses):
            (position, rotation) = self.get_odom()


            last_rotation = 0
            linear_speed = 1    #kp_distance
            angular_speed = 1  #kp_angular


            goal_x=data.pose.position.x
            goal_y=data.pose.position.y
            goal_z=math.atan2(goal_y-prev_y,goal_x-prev_x)
            # goal_z=math.atan2(1.0-1.0,-1.0-1.0)
            # print('achha',goal_z)
            # break
            # if goal_z > 180 or goal_z < -180:
            #     print("you input wrong z range.")
            #     self.shutdown()
            # goal_z = np.deg2rad(goal_z)
    
            goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
            #distance is the error for length, x,y
            self.goal_x_list.append(goal_x)
            self.goal_y_list.append(goal_y)
            self.position_x_list.append(position.x)
            self.position_y_list.append(position.y)

            distance = goal_distance
            previous_distance = 0
            total_distance = 0

            previous_angle = rotation
            total_angle = 0
            print(rotation)

            while distance > 0.15:
                (position, rotation) = self.get_odom()
                x_start = position.x
                y_start = position.y
                #path_angle = error
                path_angle = math.atan2(goal_y - y_start, goal_x- x_start)
                # print("angle",rotation,path_angle)
                # if path_angle < -pi/4 or path_angle > pi/4:
                #     if goal_y < 0 and y_start < goal_y:
                #         path_angle = -2*pi + path_angle
                #     elif goal_y >= 0 and y_start > goal_y:
                #         path_angle = 2*pi + path_angle
                # if last_rotation > pi-0.1 and rotation <= 0:
                #     rotation = 2*pi + rotation
                # elif last_rotation < -pi+0.1 and rotation > 0:
                #     rotation = -2*pi + rotation

                previous_angle = rotation
                diff_angle = path_angle - previous_angle
                print('angles',path_angle,rotation,previous_angle,diff_angle)
                diff_distance = distance - previous_distance

                distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
                # print('distance',distance)
                control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

                control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

                move_cmd.angular.z = (control_signal_angle) - rotation
                #move_cmd.linear.x = min(linear_speed * distance, 0.1)
                move_cmd.linear.x = min(control_signal_distance, 0.1)

                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 0.8)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -0.8)

                # print('my omega',move_cmd.angular.z,move_cmd.linear.x)
                last_rotation = rotation
                self.cmd_vel.publish(move_cmd)
                #self.goal_x_list.append(goal_x)
                #self.position_x_list.append(position.x)
                self.error_list.append(distance)

                r.sleep()
                previous_distance = distance
                total_distance = total_distance + distance
                prev_x=data.pose.position.x
                prev_y=data.pose.position.y
                # print("Current positin and rotation are: ", (position, rotation))

            # (position, rotation) = self.get_odom()
            # print("Current positin and rotation are: ", (position, rotation))
            # print("reached :)   ^_^")

            # while abs(rotation - goal_z) > 0.05:
            #     (position, rotation) = self.get_odom()
            #     if goal_z >= 0:
            #         if rotation <= goal_z and rotation >= goal_z - pi:
            #             move_cmd.linear.x = 0.00
            #             move_cmd.angular.z = 0.5
            #         else:
            #             move_cmd.linear.x = 0.00
            #             move_cmd.angular.z = -0.5
            #     else:
            #         if rotation <= goal_z + pi and rotation > goal_z:
            #             move_cmd.linear.x = 0.00
            #             move_cmd.angular.z = -0.5
            #         else:
            #             move_cmd.linear.x = 0.00
            #             move_cmd.angular.z = 0.5
            #     self.cmd_vel.publish(move_cmd)
            #     r.sleep()

            if c == len(path.poses)-1:
                self.shutdown()
                break
            rospy.loginfo("Stopping the robot...")
            stop_cmd.linear.x = 0.00
            stop_cmd.angular.z = 0.0
            self.cmd_vel.publish(stop_cmd)
            # rospy.spin()

        # graph

        #plt.plot(self.error_list, label='error')
        #plt.xlabel('Iteration')
        #plt.ylabel('Error')
        #plt.title('Error plot')
        #plt.legend()
        #plt.show()

        fig, ax = plt.subplots()
        # ax.plot(self.goal_x_list, label='Desired state X')
        # ax.plot(self.position_x_list, label='Current state X')
        ax.plot(self.goal_y_list, label='Desired state Y')
        ax.plot(self.position_y_list, label='Current state Y')        
        ax.legend()
        ax.set_xlabel('Step')
        # ax.set_ylabel('X position')
        ax.set_ylabel('Y position')
        plt.show()
            
        return

    # def getkey(self):
    #     global x_input, y_input, z_input
    #     x = x_input
    #     y = y_input
    #     z = z_input
    #     if x == 's':
    #         self.shutdown()
    #     x, y, z = [float(x), float(y), float(z)]
    #     return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)




# print("Please enter the lower range for generating x and y coordinates for the starting position of Turtlebot")
# lower = int(input())

# print("Please enter the upper range for generating x and y coordinates for the starting position of Turtlebot")
# upper = int(input())

# print("Random initial X and Y coordinates of the Turtlebot are:-")

# coord = np.array[[0.0],[0.0]]
# print('Initial X coordinate: ', coord[0])
# print('Initial Y coordinate: ', coord[1])

# print("Please enter the lower range for generating angle wrt x axis for the starting position of Turtlebot in degrees")
# lower_angle = math.radians(int(input()))

# print("Please enter the upper range for generating angle wrt x axis for the starting position of Turtlebot in degrees")
# upper_angle = math.radians(int(input()))

# angle = np.array[0.0]  
# print('Initial starting angle Theta wrt +X axis: ', angle[0])   

# initial_position = coord + angle
initial_position = np.array([0.0,0.0,0.0])

# #print('(X, Y, Theta):' ,coord[0], coord[1], angle[0])
# print('Initial pose is:-')
# print('(X, Y, Theta):', initial_position[0], initial_position[1], initial_position[2])

# print("Enter final x position")
# x_final = input()
# print("Enter final y position")
# y_final = input()
# print("Enter final angle position")
# angle_final = input()

# final = [x_final, y_final, angle_final]
# final_position = np.array(final)

# x_input = final_position[0]
# y_input = final_position[1]
# z_input = final_position[2]


q = quaternion_from_euler(0, 0, initial_position[2])
# state_msg is an object
state_msg = ModelState()
state_msg.model_name = 'turtlebot3_burger'
state_msg.pose.position.x = initial_position[0]
state_msg.pose.position.y = initial_position[1]
state_msg.pose.position.z = 0.3

state_msg.pose.orientation.x = q[0]
state_msg.pose.orientation.y = q[1]
state_msg.pose.orientation.z = q[2]
state_msg.pose.orientation.w = q[3]

set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
resp = set_state(state_msg)
print(resp)

time.sleep(5)


GotoPoint()