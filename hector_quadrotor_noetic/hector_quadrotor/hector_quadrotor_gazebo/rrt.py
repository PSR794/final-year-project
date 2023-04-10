#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid , Odometry , Path
from geometry_msgs.msg import PoseStamped
import time as TIME
import random
import cv2 as cv
import numpy as np
from skimage.draw import line as lne
print('working')
drawing = cv.imread('/home/psr/FYP/src/map/drone_map.pgm')
font = cv.FONT_HERSHEY_SIMPLEX


def attach_node(p1,p2,min_d):
    radius = 30
    p1p2 = p2 - p1
    mag_p1p2 = p1p2/min_d
    p3 = p1 + radius*mag_p1p2
    # print(p3)
    p3 = np.array(p3,dtype=np.int32)
    return p3


def find_nearest_node(distance,nodes,p2):
    distance[:,:2]  = (nodes - p2)**2
    distance[:,2]  = distance[:,0] + distance[:,1]
    min_ind = np.where(distance[:,2] == np.min(distance[:,2]))[0][0]
    x1 = nodes[min_ind][0]
    y1 = nodes[min_ind][1]
    return x1 , y1 , np.min(distance[:,2])**0.5,min_ind

def check_collision(p1,p3,map_binary):
    discrete_line = list(zip(lne(p1[0,1],p1[0,0],p3[0,1],p3[0,0] )))
    discrete_line = np.vstack((discrete_line[0],discrete_line[1])).T
    return 0 in map_binary[discrete_line[:,1],discrete_line[:,0]]

def main():
    rospy.init_node('planner',anonymous=True)
    pub_path = rospy.Publisher('/path_rrt',Path,queue_size=10)
    map = rospy.wait_for_message('/map',OccupancyGrid)

    h = map.info.height
    w = map.info.width
    map_x = map.info.origin.position.x
    map_y = map.info.origin.position.y
    res = map.info.resolution
    print(len(map.data),h,w,map_x,map_y,res)
    map_image = np.array(map.data,dtype=np.uint8).reshape((h,w))
    map_image = cv.flip(map_image,1)
    map_copy = map_image.copy()

    _,map_binary = cv.threshold(map_copy,10,255,cv.THRESH_BINARY)

    map_binary = abs(map_binary - 255) * 255
    map_binary = cv.resize(map_binary,dsize=(0,0),fx=1,fy=1)
    drawing2 = map_binary.copy()
    
    print(type(map_binary[0,0]))

    search_space = np.vstack( (np.where(map_binary == 255)[0],np.where(map_binary == 255)[1]) ).T
    print(search_space.shape)
    goal = rospy.wait_for_message('/move_base_simple/goal',PoseStamped)
    goal_x = int((goal.pose.position.y - map_y)/res)
    goal_y = int((goal.pose.position.x - (w*res) - map_x)/(-res))
    init_x , init_y = 700 , 612 #150 , 108  #250 , 190
    nodes = np.array([[init_x,init_y]])
    cv.circle(map_binary,(init_y,init_x),6,(255,0,0),-1)
    # cv.imshow('map',map_binary)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    distance = np.zeros((nodes.shape[0],nodes.shape[1]+1))
    goal = np.array([[goal_x,goal_y]],dtype = np.int32)
    print('goal',goal)
    parents = [0]
    radius = 30
    start_time = TIME.time()
    p3 = np.array([[init_x,init_y]],dtype=np.int32)
    d = (np.sum((p3 - goal)**2))**0.5
    print(d)
    while d > 50:
        radius = 30
        rand_pixel = random.randint(0, search_space.shape[0]-1)
        x2 = search_space[rand_pixel][0]
        y2 = search_space[rand_pixel][1]
        p2 = np.array([[x2,y2]],dtype=np.int32)

        x1 , y1 , min_d , min_ind= find_nearest_node(distance,nodes,p2)

        p1 = np.array([[x1,y1]],dtype=np.int32)
        # print(p1)
        if min_d < radius:
            p3 = p2

        else:
            p3 = attach_node(p1,p2,min_d)

        blocked_path = check_collision(p1,p3,map_binary)
        
        if blocked_path:
            continue

        if not blocked_path:
            # parents.append( len(nodes)-1 )
            x3 , y3 = p3[0,0] , p3[0,1]    
            p2 = np.array([x2,y2]).reshape(1,2)
            nodes = np.vstack((nodes,p3))
            distance = np.zeros((nodes.shape[0],nodes.shape[1]+1))

            parents.append(min_ind)


        blocked_path = check_collision(p3,goal,map_binary)

        if not blocked_path:
            parents.append( len(nodes)-1 )
            nodes = np.vstack((nodes,goal))
            print(nodes)
            break

    node = len(nodes)-1
    z = 0
    path_x , path_y= [],[]

    while node != 0:
        x_c , y_c = nodes[node][0] , nodes[node][1]

        path_x.append(x_c)
        path_y.append(y_c)
        node = parents[node]

        x_p , y_p = nodes[node][0] , nodes[node][1]
        path_x.append(x_p)
        path_y.append(y_p)
        z+=1
    raxy = np.array(np.vstack((path_x,path_y)).T,dtype=np.float)
    raxy[:,1] = (((-1*raxy[:,1])+ w)*res) + map_x  #(raxy[:,1]*(-res)) + map_x + (w*res)
    raxy[:,0] = (raxy[:,0]*res) + map_y
    print(raxy.shape,'rviz')
    way = Path()
    way.header.seq = 1
    way.header.stamp = rospy.Time.now()
    way.header.frame_id = 'map'
    
    for i in range(raxy.shape[0]):
        way_points = PoseStamped()
        way_points.header = way.header
        way_points.pose.position.x = raxy[i,1]
        way_points.pose.position.y = raxy[i,0]
        print(way_points.pose.position.x,way_points.pose.position.y)
        way.poses.insert(0,way_points)
    print('\ndone',TIME.time() - start_time)
    while not rospy.is_shutdown():
        pub_path.publish(way)
    

if __name__ == "__main__":
    main()