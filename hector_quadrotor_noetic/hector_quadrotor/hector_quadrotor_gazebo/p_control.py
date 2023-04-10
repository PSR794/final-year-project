#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid , Odometry , Path
from turtlesim.msg import Pose
import math
def pose_callback(pose_msg):
    global goal_x, goal_y
    print("amcl pose")
    print(pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y)
    while (True):
        #dist=abs(math.sqrt(((goal_x-pose_msg.x)**2)+((goal_y-pose_msg.y)**2)))
        dist_diff=dist_cal(goal_x,goal_y,pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y)
        #angle=math.atan2(goal_y-pose_msg.y,goal_x-pose_msg.x)
        angle=ang_cal(goal_x,goal_y,pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y)
        vel_msg.linear.x= 0.5 * (dist_diff)
        vel_msg.linear.y = 0
        vel_msg.linear.z= 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 3 * (angle-pose_msg.theta)
        velocity_publisher.publish(vel_msg)
        print(pose_msg)
        #loop_rate.sleep()
        if (dist_diff<0.01):
            #vel_msg.linear.x=0
            #vel_msg.angular.z=0
            #velocity_publisher.publish(vel_msg)
            #print(pose_msg)
            break
    pose_msg.x=pose_message.x
    pose_msg.y=pose_message.y
    pose_msg.theta=pose_message.theta
    pose_msg.linear_velocity=pose_message.linear_velocity
    pose_msg.angular_velocity=pose_message.angular_velocity
    print(pose_msg)

def dist_cal(x1,y1,x2,y2):
    dist=abs(math.sqrt(((x1-x2)**2)+((y1-y2)**2)))
    return dist

def ang_cal(x1,y1,x2,y2):
    angle_cal=math.atan2(y1-y2,x1-x2)
    return angle_cal

def move():
    global pose_msg,goal_x, goal_y,vel_msg, velocity_publisher
    rospy.init_node('onemotion_try', anonymous=True)
    data=rospy.wait_for_message('/path_rrt',Path)
    velocity_publisher=rospy.Publisher('/tbot/cmd_vel',Twist,queue_size=10)
    pose_subscriber=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,pose_callback)
    
    for i in data.poses:
        print(i.pose.position.x ,i.pose.position.y)    
    
    vel_msg=Twist()
    pose_msg=Pose()
    goal_x=data.poses[3]
    goal_y=data.poses[3].pose.position.y

    

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass