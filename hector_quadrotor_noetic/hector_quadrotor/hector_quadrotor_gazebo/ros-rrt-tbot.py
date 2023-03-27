#!/usr/bin/env python3

import rospy 
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import math 
from time import time


global map,res,height,width,arr,r
print('Starting...')
rospy.init_node('RRT_algo', anonymous=True)
map = rospy.wait_for_message("/map",OccupancyGrid)

res = map.info.resolution
height = map.info.height
width = map.info.width
conv = np.array([[1,1,1,1,1],[1,2,2,2,1],[1,2,10,2,1],[1,2,2,2,1],[1,1,1,1,1]]) # GAUSSIAN KERNEL
r = rospy.Rate(1)
pub = rospy.Publisher('/path_to_goal',Path,queue_size=10)
explore= rospy.Publisher('/path_to_goal',Path,queue_size=10)

#TO CALCULATE DISTANCE BETWEEN TWO POINTS
def dist_cal(x1,y1,x2,y2):
    dist=abs(math.sqrt(((x1-x2)**2)+((y1-y2)**2)))
    return dist

#RANDOM SAMPLING
def initializer(h):
    init=np.random.randint([0,0],[1312,1312])
    return init,h 

#CONVOLUTION
def convol(arr,conv):
    x = arr.shape[0]
    y = arr.shape[1]
    
    temp = np.ones((x+4,y+4))*100
    temp[2:x+2,2:y+2] = arr.copy()
    t2 = np.ones((x,y))*100

    for i in range(2,x):

        for j in range(2,y):
            t2[i-2,j-2] = np.sum(temp[i-2:i+3,j-2:j+3]*conv)/np.sum(conv)

    return t2 

def extrapolate(conv,data,val):
    a= np.array(data.data).reshape(height,width).T#[1511:1825,1322:2010]
    for i in range(val):
        a[a <= 5] = 5
        a = convol(a,conv)
    a[a >= 95] = 1000
    return a

#FIND A NODE IN THE EXISTING TREE WHICH IS CLOSEST TO THE RANDOM POINT
def find_closest(c,d):
    global nodes
    N_c=np.array([c,d])
    d=nodes-N_c
    d=np.square(d)
    d[:,0]=d[:,0]+d[:,1]
    d0=np.min(d[:,0])
    p=np.where(d[:,0]==d0)
    z=nodes[p]
    return z[0][0],z[0][1]

# TO AVOID OBSTACLES IN THE MAP
def collision_avoidance(arr,h1,k1,h2,k2):
    g= True
    #LINEAR INTERPOLATION
    for i in range(1,20):
        t=i/20
        x=h1*t+h2*(1-t)
        y=k1*t+k2*(1-t)
        if arr[int(x)][int(y)] ==1000:                       
            g=False
            print("g is",g)
            break

    return g 

#TO FIND A POINT THAT IS AT A DISTANCE d FROM THE NODE AND ALONG THE LINE JOINING THE RANDOM POINT AND THE NODE
def find_point(a1,b1,a2,b2,d): 
    m=(b2-b1)/(a2-a1)
    x1= (d/math.sqrt(1+m**2))+a1
    y1= b1+(d*m/math.sqrt(1+m**2))
    x2= (-d/math.sqrt(1+m**2))+a1
    y2= b1-(d*m/math.sqrt(1+m**2))
    ran_dist=dist_cal(a1,b1,a2,b2)
    d1=dist_cal(a2,b2,x1,y1)
    d2=dist_cal(a2,b2,x2,y2)
    if d1<ran_dist:
        x,y=x1,y1
    else:
        x,y=x2,y2

    return int(x),int(y)

# TO RETRIEVE THE FINAL PATH TO THE GOAL AFTER EXPLORATION
def get_path(ip_x,ip_y, gp_x,gp_y, predecessors,nodes):
    global route
    route = Path()
    route.header.seq = 1
    route.header.stamp = rospy.Time.now()
    route.header.frame_id = 'map'

    m=gp_x
    n=gp_y
    po = PoseStamped()
    po.header = route.header
    po.pose.position.x = res*(m - arr.shape[0]/2 )
    po.pose.position.y = res*(n - arr.shape[1]/2 )
    route.poses.insert(0,po)
    path=np.array([[m,n]])
    while (m!=ip_x) & (n!=ip_y):
        j=np.where((nodes[:,0]==m) & (nodes[:,1]==n))
        z=parents[j]
        m=z[0][0]
        n=z[0][1]
        po = PoseStamped()
        po.header = route.header
        po.pose.position.x = res*(m - arr.shape[0]/2 )
        po.pose.position.y = res*(n - arr.shape[1]/2 )
        route.poses.insert(0,po)
    
    po = PoseStamped()
    po.header = route.header
    po.pose.position.x = res*(m - arr.shape[0]/2 )
    po.pose.position.y = res*(n - arr.shape[1]/2 )

    route.poses.insert(0,po)
    return route

#INITIALIZING THE FIRST NODE
def first_node(arr,x_start,y_start):
    global nodes, parents
    shape=arr.shape
    print("shape is",shape)

    node_init,_=initializer(1)
    
    if arr[node_init[0]][node_init[1]] == 5:
        node_x,node_y = find_point(x_start,y_start,node_init[0],node_init[1],20)
        print("first node",node_x,node_y)
        nodes=np.concatenate((nodes, np.array([[node_x,node_y]])), axis=0)
        parents=np.concatenate((parents, np.array([[x_start,y_start]])), axis=0)

    else:
        first_node(arr,x_start,y_start)


def new_node(arr,n,g_x,g_y):
    global nodes, parents
    i=1

    while i<n+1:
        node,_=initializer(i)        
        x_close,y_close= find_closest(node[0],node[1])
        if x_close != node[0]: #TO AVOID INFINITE SLOPE
            node_fin_x,node_fin_y=find_point(x_close,y_close,node[0],node[1],20)

            if x_close-node_fin_x!=0:
                if collision_avoidance(arr,x_close,y_close,node_fin_x,node_fin_y)==True:
                    if arr[node_fin_x][node_fin_y] ==5: #TO CHECK IF THE POINT IS UNOCCUPIED
                        g_dist=dist_cal(g_x,g_y,node_fin_x,node_fin_y)
                        if g_dist< 25:
                            #TO END THE TREE IF GOAL IS NEARBY
                            nodes=np.concatenate((nodes, np.array([[node_fin_x,node_fin_y]]),np.array([[g_x,g_y]])), axis=0)
                            parents=np.concatenate((parents, np.array([[x_close,y_close]]),np.array([[node_fin_x,node_fin_y]])), axis=0)
                            print("GOAL REACHED!!!")
                            break
                        else:
                            nodes=np.concatenate((nodes, np.array([[node_fin_x,node_fin_y]])), axis=0)
                            parents=np.concatenate((parents, np.array([[x_close,y_close]])), axis=0)
                            i=i+1


def planner(arr,i_x,i_y,g_x,g_y):
    global nodes, parents
    nodes=np.array([[i_x,i_y]])
    parents =np.array([[i_x,i_y]])
    first_node(arr,i_x,i_y)
    number=700

    #RRT exploration tree
    new_node(arr,number,g_x,g_y)
    return get_path(i_x,i_y,g_x,g_y,parents,nodes)

#CALLBACK FUNCTION
def pose_cb(data,goal):  
    global map,res,height,width,arr,r
    print('Goal Received')
    arr = extrapolate(conv,map,5)
    print('Map Ready!')
    init_x = int(data.pose.pose.position.x/res + arr.shape[0]/2)
    init_y = int(data.pose.pose.position.y/res + arr.shape[1]/2)



    goal_x = int(goal.pose.position.x/res + arr.shape[0]/2)
    goal_y = int(goal.pose.position.y/res + arr.shape[1]/2)

    print("initial",arr[init_x][init_y])
    print('goal = ',arr[goal_x][goal_y])

    begin = time()
    print('Planning Started')
    pub.publish(planner(arr,init_x,init_y, goal_x,goal_y))
    end = time()
    print('Time = ', end - begin)
    
    r.sleep()

def main():

    odomSub = Subscriber('/tbot/ground_truth/state',Odometry)
    goalSub = Subscriber('/move_base_simple/goal',PoseStamped)

    ats = ApproximateTimeSynchronizer(
    [odomSub, goalSub], 100, 100, allow_headerless=True
    )
    ats.registerCallback(pose_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass