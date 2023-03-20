import numpy as np
import cv2
import time 
import heapq as hq
import matplotlib.pyplot as plt

class node_object:
    def __init__(self,pt,cost,parent_node,theta,cost_to_goal) :
        self.pt=pt
        self.cost=cost 
        self.parent_node=parent_node
        self.theta=theta
        self.cost_to_goal=cost_to_goal
    def __lt__(self,other):
        return self.cost+self.cost_to_goal < other.cost+other.cost_to_goal

def get_input():
    start_coor=tuple(int(item) for item in input("\n Start Node: ").split(','))
    start_theta=int(input("Enter Start Orientation ")) 
    end_coor= tuple(int(item) for item in input("\n End Node: ").split(','))
    end_theta=int(input("Enter Goal Orientation:")) 
    step_size=int(input("Enter the Step Size:")) 
    if step_size>10:
        print ("Step Size Too Large, Exiting.PLease run program again")
        exit()
    robot_radius=int(input("Enter the Robot Radius:")) 
    clr=int(input("Enter the Clearence:"))
    return start_coor,start_theta,end_coor,end_theta,step_size,robot_radius,clr

def generate_map(clearence):
    #NEED TO UPDATE COLOR 
    canvas = np.zeros((250,600,3),dtype="uint8") 
    # canvas[:, :] = (255,255,255)

    white=(255,255,255)
    black=(0,0,0)
    blue=(255,0,0)
    pts_bloated_map=[np.array([[0+clearence,0+clearence],[600-clearence,0+clearence],[600-clearence,250-clearence],[0+clearence,250-clearence]])]
    cv2.fillPoly(canvas , pts_bloated_map, color=(5,5,5)) 

    #Triangle
    pts_t = [np.array([[460,25],[510,125], [460,225]])]
    cv2.fillPoly(canvas , pts_t, color=black) 
    cv2.polylines(canvas,pts_t, True ,color=white,thickness=clearence)

    #Hexagon
    pts_h = [np.array([[235.04, 87.5], [235.05, 162.5], [300, 200], [364.95, 162.5], [364.95, 87.5], [300, 50], [235.04, 87.5]],np.int32)]
    cv2.fillPoly(canvas, pts_h, color=black)
    cv2.polylines(canvas,pts_h,True,color=white,thickness=clearence)

    #RECT 1 
    cv2.rectangle(canvas,(100-clearence,0+clearence),(150+clearence,100+clearence),color=white,thickness=-1)
    cv2.rectangle(canvas,(100,0+clearence),(150,100),color=black,thickness=-1)
    # #RECT 2
    cv2.rectangle(canvas,(100-clearence,150-clearence),(150+clearence,250-clearence),color=white,thickness=-1)
    cv2.rectangle(canvas,(100,150),(150,250-clearence),color=black,thickness=-1)

    return canvas