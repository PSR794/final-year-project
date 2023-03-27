#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import cv2
import math 

#TO CALCULATE DISTANCE BETWEEN TWO POINTS
def dist_cal(x1,y1,x2,y2):
    dist=abs(math.sqrt(((x1-x2)**2)+((y1-y2)**2)))
    return dist

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

#AVOID COLLISION
def collision_avoidance(gmap,h1,k1,h2,k2):
    g= True
    #LINEAR INTERPOLATION
    for i in range(1,20):
        t=i/20
        x=h1*t+h2*(1-t)
        y=k1*t+k2*(1-t)
        if gmap[int(y),int(x)] ==0:                       
            g=False
            print("g is",g)
            break

    return g

#INITIALIZING THE FIRST NODE
def first_node(image,colour_image,x_start,y_start):
    global nodes, parents,parents_dict
    shape=image.shape
    node_init,_=initializer(1)
    
    if image[node_init[1],node_init[0]] == 254:
        node_x,node_y = find_point(x_start,y_start,node_init[0],node_init[1],20)
        cv2.line(colour_image,(x_start,y_start),(node_x,node_y),(255,0,0),1)
        cv2.circle(colour_image,(node_x,node_y),3 , (0,0,255), 1)
        nodes=np.concatenate((nodes, np.array([[node_x,node_y]])), axis=0)
        parents=np.concatenate((parents, np.array([[x_start,y_start]])), axis=0)

    else:
        first_node(image,colour_image,x_start,y_start)

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

#RANDOM SAMPLING
def initializer(h):
    init=np.random.randint([0,0],[1312,1312])
    return init,h 


def new_node(map,colour_map,n,g_x,g_y):
    global nodes, parents
    i=1
    shape=map.shape
    while i<n+1:
        node,l=initializer(i)        
        x_close,y_close= find_closest(node[0],node[1])
        if x_close != node[0]: #TO AVOID INFINITE SLOPE
            node_fin_x,node_fin_y=find_point(x_close,y_close,node[0],node[1],20)

            if x_close-node_fin_x!=0:
                if collision_avoidance(map,x_close,y_close,node_fin_x,node_fin_y)==True:
                    if map[node_fin_y,node_fin_x] ==254: #TO CHECK IF THE POINT IS UNOCCUPIED
                        g_dist=dist_cal(g_x,g_y,node_fin_x,node_fin_y)
                        if g_dist< 25:
                            #TO END THE TREE IF GOAL IS NEARBY
                            cv2.line(colour_map,(g_x,g_y),(node_fin_x,node_fin_y),(255,0,0),1)
                            cv2.circle(colour_map,(node_fin_x,node_fin_y),3 , (0,0,255), 1)
                            cv2.line(colour_map,(x_close,y_close),(node_fin_x,node_fin_y),(255,0,0),1)
                            nodes=np.concatenate((nodes, np.array([[node_fin_x,node_fin_y]]),np.array([[g_x,g_y]])), axis=0)
                            parents=np.concatenate((parents, np.array([[x_close,y_close]]),np.array([[node_fin_x,node_fin_y]])), axis=0)
                            print("GOAL REACHED!!!")
                            break
                        else:
                            cv2.circle(colour_map,(node_fin_x,node_fin_y),3 , (0,0,255), 1)
                            cv2.line(colour_map,(x_close,y_close),(node_fin_x,node_fin_y),(255,0,0),1)        
                            nodes=np.concatenate((nodes, np.array([[node_fin_x,node_fin_y]])), axis=0)
                            parents=np.concatenate((parents, np.array([[x_close,y_close]])), axis=0)
                            i=i+1
                            
#RETRIEVE THE FINAL PATH FROM THE TREE
def final_path(path_map):
    global nodes,parents,goal_x,goal_y,begin_x,begin_y

    m=goal_x
    n=goal_y
    path=np.array([[m,n]])
    while True:
        j=np.where((nodes[:,0]==m) & (nodes[:,1]==n))
        z=parents[j]
        cv2.circle(path_map,(m,n),3 , (0,0,255), 1)
        cv2.line(path_map,(m,n),(z[0][0],z[0][1]),(255,0,0),1)
        m=z[0][0]
        n=z[0][1]

        path=np.concatenate((path, np.array([[m,n]])), axis=0)
        if (m==begin_x) & (n==begin_y):

            cv2.circle(path_map,(m,n),7 , (0,255,0), 3)
            break
 
def main():
    global nodes,parents,goal_x,goal_y,begin_x,begin_y

    #LOADING THE MAP
    img = cv2.imread("/home/psr/FYP/src/map/drone_map.pgm")
    colour_img= cv2.imread("/home/psr/FYP/src/map/drone_map.pgm")
    path_img=cv2.imread("/home/psr/FYP/src/map/drone_map.pgm")
    # img = img[1218:2050,1470:1843]
    # colour_img = colour_img[1218:2050,1470:1843]
    # path_img=path_img[1218:2050,1470:1843]
    print(img.shape)
    # cv2.imshow("RRT Algorithm",img)
    # cv2.waitKey(0)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #ADD GAUSSIAN BLUR TO THE MAP TO MAINTAIN SAFE DISTANCE FROM THE OBSTACLES
    img = cv2.GaussianBlur(img,(23,23),cv2.BORDER_DEFAULT)

    #START LOCATION
    begin_x=int(input("enter x coordinate of start position:"))
    begin_y=int(input("enter y coordinate of start position:"))
    nodes=np.array([[begin_x,begin_y]])
    cv2.circle(colour_img,(begin_x,begin_y),3 , (0,0,255), 1)

    #GOAL LOCATION
    goal_x=int(input("enter x coordinate of goal position:"))
    goal_y=int(input("enter y coordinate of goal position:"))    
    cv2.circle(colour_img,(goal_x,goal_y),7 , (0,255,0), 3)
    parents =np.array([[begin_x,begin_y]])

    #INITIALIZING FIRST RANDOM NODE
    first_node(img,colour_img,begin_x,begin_y)
    number=700

    #RRT exploration tree
    new_node(img,colour_img,number,goal_x,goal_y)

    #Path found
    final_path(path_img)
    Hori = np.concatenate((colour_img,path_img), axis=1)
    cv2.imshow("RRT Algorithm",Hori)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

main()