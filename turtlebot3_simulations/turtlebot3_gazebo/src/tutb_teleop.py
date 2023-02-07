#!/usr/bin/env python3

#Teleoperation
#Left arrow key - robot moves left
#right arrow key - robot moves right
#up arrow key - robot moves forward
#down arrow key - robot moves back
#Enter - robot stops
#make sure you install the pynput library

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from pynput import keyboard
from pynput.keyboard import Listener, Key
from geometry_msgs.msg import Twist
PI = np.pi


class turtlebot3:
    def __init__(self):

        rospy.init_node('turtle_controller', anonymous=True)


        self.wheel_mode_time = 3.25 # secs            
        self.publisher = rospy.Publisher('/tbot/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(100)
        self.speed = Twist()
    # def torque_callback(self, data):
    #     self.data = data

    def move(self):
        self.start = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            with keyboard.Listener(on_press=bot.on_press,on_release=on_release) as listener:
                listener.join()
       
    def on_press(self,key):
        try:
            if key == Key.up:
                print('Moving Forward')
                self.now = rospy.get_rostime()

                self.speed.linear.x  = 0.5
                # self.speed.angular.z  = 0.0
                self.publisher.publish(self.speed) # RPM          
                self.rate.sleep()

            elif key== Key.left:
                print('Moving left')                
                self.now = rospy.get_rostime()

                self.speed.angular.z  = 0.5
                self.speed.linear.x = 0.0
                self.publisher.publish(self.speed) # RPM                             
                self.rate.sleep()    

            elif key==Key.right:
                print('Moving right')               
                self.now = rospy.get_rostime()

                self.speed.angular.z  = -0.5   
                self.speed.linear.x = 0.0           
                self.publisher.publish(self.speed) # RPM               
                self.rate.sleep()    

            elif key==Key.down:
                print('Moving back')
                self.now = rospy.get_rostime()

                self.speed.linear.x = -0.5
                self.speed.angular.z  = 0.0
                self.publisher.publish(self.speed) # RPM                
                self.rate.sleep()

            elif key==Key.enter:
                print('Stop')
                self.now = rospy.get_rostime()

                self.speed.linear.x = 0.0
                self.speed.angular.z  = 0.0                
                self.publisher.publish(self.speed) # RPM               
                self.rate.sleep()

            elif key==Key.esc:
                rospy.signal_shutdown("Shutting down")
            else:
                print('Invalid key, robot retains previous command')
        except AttributeError:
            print('Invalid Key, robot retains previous command')

def on_release(key):
    print(key) 
    if key == keyboard.Key.esc:
        return False


           
if __name__ == '__main__':
    bot = turtlebot3()
    bot.move()
