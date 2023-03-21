# GITHUB LINK - https://github.com/theunknowninfinite/ENPM661_project_3

# DONE BY 
# Jayasuriya Suresh
# Aashrita Chemakura


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

#Getting inputs
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

    white=(0,0,255)
    black=(0,0,0)
    blue=(255,0,0)
    pts_bloated_map=[np.array([[0+clearence,0+clearence],[600-clearence,0+clearence],[600-clearence,250-clearence],[0+clearence,250-clearence]])]
    cv2.fillPoly(canvas , pts_bloated_map, color=(253,253,253)) 

    #Triangle
    pts_t = [np.array([[460,25],[510,125], [460,225]])]
    cv2.fillPoly(canvas , pts_t, color=white) 
    cv2.polylines(canvas,pts_t, True ,color=black,thickness=clearence)

    #Hexagon
    pts_h = [np.array([[235.04, 87.5], [235.05, 162.5], [300, 200], [364.95, 162.5], [364.95, 87.5], [300, 50], [235.04, 87.5]],np.int32)]
    cv2.fillPoly(canvas, pts_h, color=white)
    cv2.polylines(canvas,pts_h,True,color=black,thickness=clearence)

    #RECT 1 
    cv2.rectangle(canvas,(100-clearence,0+clearence),(150+clearence,100+clearence),color=black,thickness=-1)
    cv2.rectangle(canvas,(100,0+clearence),(150,100),color=white,thickness=-1)
    # #RECT 2
    cv2.rectangle(canvas,(100-clearence,150-clearence),(150+clearence,250-clearence),color=black,thickness=-1)
    cv2.rectangle(canvas,(100,150),(150,250-clearence),color=white,thickness=-1)

    return canvas

#Function to check if the input is in the obstacle
def check_if_in_obstacle(point,canvas):
    x,y=point

    if all(val == 255 for val in canvas[y][x]):
        return True

    elif all(val == 0 for val in canvas[y][x]):
        return True    

    else:
        print("Point  Clear")
        return False

#Checking if the given input coordinates are valid
def check_valid_points(point,clr,canvas):
    x,y=point
    if x<(0+clr) or x>(600-clr) or y<(0+clr) or y>(250-clr):
        print("Point not in region")
        valid_point=False
        return valid_point

    
    flag_ob=check_if_in_obstacle(point,canvas)
    if flag_ob:
        print("Point in Obstruction in FN Check valid Point")
        valid_point=False
        return valid_point
    
    else:
        valid_point = True
        return valid_point
        
def node_id(node):
    x,y=node.pt
    key = 1022*x + 111*y 
    return key

#Defining actions
def actionset(point,cost,theta,step_size,clr,canvas):
    px,py=point
    cost=cost
    
    list_of_actions=[]
    #angle,cost
    actions=[[theta+60,1],[theta+30,1],[theta+0,1],[theta-30,1],[theta-60,1],]
   
    for idx in actions:
        x=px+np.cos(np.deg2rad(idx[0]))*step_size
        y=py+np.sin(np.deg2rad(idx[0]))*step_size
        cost1= cost+idx[1]
        x=int(np.round(x,0))
        y=int(np.round(y,0))
        
        if (check_valid_points((x,y),clr,canvas)==True):
            list_of_actions.append(((x,y),idx[0],cost1))

    return list_of_actions

#Calculating the heuristic distance
def euclidiean_distance(point1,point2):
    x1,y1=point1
    x2,y2=point2
    distance=np.sqrt((x2-x1)**2+(y2-y1)**2)
    return distance

#A-Star algorithm
def astar(start_node, end_node, canvas, step_size, clr):
    start = start_node
    end = end_node
    goal_dist_thresh = 1.5
    nodes_to_explore = {node_id(start): start}
    nodes_explored = {}
    queue_of_nodes = [(start.cost, start)]
    all_nodes_list = []

    while queue_of_nodes:
        current_node = hq.heappop(queue_of_nodes)[1]
        # all_nodes_list.append([current_node.pt, current_node.theta])
        all_nodes_list.append((current_node.pt))
        current_node_id = node_id(current_node)
        dt = euclidiean_distance(current_node.pt, end.pt)
        if dt < goal_dist_thresh:
            end_node.parent_node = current_node.parent_node
            end_node.cost=current_node.cost
            print("GOAL REACHED")
            return all_nodes_list, 1
        if current_node_id in nodes_explored:
            continue
        else:
            nodes_explored[current_node_id] = current_node
        nodes_to_explore.pop(current_node_id, None)
        
        actions = actionset(current_node.pt, current_node.cost, current_node.theta, step_size, clr, canvas)
        for idx in actions:
            x, y = idx[0]
            theta = idx[1]
            cost = idx[2]
            cost_to_go = euclidiean_distance((x, y), end.pt)
            new_node = node_object((x, y), cost, current_node, theta, cost_to_go)
            new_node_id = node_id(new_node)
            if not check_valid_points(new_node.pt, clr, canvas) or new_node_id in nodes_explored:
                continue
            if new_node_id in nodes_to_explore and new_node.cost < nodes_to_explore[new_node_id].cost: 
                nodes_to_explore[new_node_id] = new_node
            else:
                nodes_to_explore[new_node_id] = new_node
            hq.heappush(queue_of_nodes, (new_node.cost + new_node.cost_to_goal, new_node))

    return all_nodes_list, 0

#doing backtracking 
def back_tracking(goal_node):  
    path_taken = []
    path_taken.append(goal_node.pt)
    parent_n = goal_node.parent_node

    while parent_n != None:
        path_taken.append(parent_n.pt)
        parent_n = parent_n.parent_node
        
    path_taken.reverse()
    path_taken = np.asarray(path_taken)

    return path_taken

#plotting Points 
def alt_plot_fn(canvas,all_nodes_list,backtrack_nodes):
    color_path=(0,255,0)
    counter=0
    color_nodes=(255,0,0)
    # print(canvas.shape)
    height= canvas.shape[0]
    cv2.imshow('Map', canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    for i in all_nodes_list:
                x,y=i
                cv2.circle(canvas,(x,height-y),1,color_nodes,1)
                canvas[height-y][x]=color_nodes
                counter += 1
                if counter == 100:
                    counter = 0
                    cv2.imshow('Path', canvas)
                    cv2.waitKey(1)
    print("Backtracking Nodes")
    for i in backtrack_nodes:
                canvas[height- i[1],i[0]]= color_path
                counter += 1
                # print(y,x)
                if counter == 10:
                    counter = 0
                    cv2.imshow('Path', canvas)
                    cv2.waitKey(1)

    cv2.imshow('Path', canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



#calling all functions to get input , check inputs and plot the final results 
start_coor,start_theta,end_coor,end_theta,step_size,robot_radius,clearence=get_input()
start_time=time.time()
img=generate_map(robot_radius+clearence)

#checking points
if not check_valid_points(start_coor,clearence+robot_radius,img):
    print("Wrong Start Node ,Please run program again and enter proper value")
    exit()
if not check_valid_points(end_coor,clearence+robot_radius,img):
    print("Wrong End Node, Please run program again and enter proper value")
    exit()
if end_coor == start_coor:
    print ("Start and End are the Same Exiting.PLease run program again")
    exit()
start = node_object(start_coor, 0, None, start_theta, 0)
end = node_object(end_coor, 0, None, end_theta, 0)

list_of_nodes, flag = astar(start, end, img, step_size, clearence+robot_radius)
end_time=time.time()-start_time
print("Code Took", end_time,"Seconds")

#plotting
if flag == 1:
    path_taken = back_tracking(end)
    alt_plot_fn(img,list_of_nodes,path_taken)
else:
    print("Path could not be found")

