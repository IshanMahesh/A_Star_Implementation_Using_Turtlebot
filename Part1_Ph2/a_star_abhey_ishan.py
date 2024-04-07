### GitHub repository link
# https://github.com/IshanMahesh/A-star-Algorithm-for-a-Mobile-Robot.git
###

#Import important libraries 

import time
import cv2
import heapq as hq
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import imutils


def obstacle_map(canvas):

    # creating boundary
    cv2.rectangle(canvas, (0, 0), (width, height), (0, 0, 255), -1)
    cv2.rectangle(canvas, (0+total_clearance, 0+total_clearance), (width-total_clearance, height-total_clearance), (255, 255, 255), -1)

    #Creating rectangle 1
    cv2.rectangle(canvas, pt1=(1750//scaling_factor + total_clearance,1000//scaling_factor -total_clearance),pt2=(1500//scaling_factor - total_clearance,2000//scaling_factor + total_clearance), color=(0,0,255), thickness=-1)
    cv2.rectangle(canvas,pt1=(1750//scaling_factor ,1000//scaling_factor),pt2=(1500//scaling_factor ,2000//scaling_factor),color=(179,41,43),thickness=-1)

    #Creating rectangle 2
    cv2.rectangle(canvas,pt1=(2500//scaling_factor - total_clearance,1000//scaling_factor + total_clearance),pt2=(2750//scaling_factor + total_clearance,0 - total_clearance),color=(0,0,255),thickness=-1)
    cv2.rectangle(canvas,pt1=(2500//scaling_factor, 1000//scaling_factor),pt2=(2750//scaling_factor ,0),color=(179,41,43),thickness=-1)
    
    #Creating a Circle with the clearance
    center = (4200//scaling_factor, 1200//scaling_factor)
    radius = 600//scaling_factor + total_clearance
    cv2.circle(canvas, center, radius, (0, 0, 255), -1)

    # Draw the original circle
    radius = 600//scaling_factor
    cv2.circle(canvas, center, radius, (179, 41, 43), -1)
    return canvas




def input_coordinates():

    #check if start and goal nodes are valid
    while True:
        start_node_str = input("Enter the coordinates of starting node (x,y,theta):")
        goal_node_str = input("Enter the coordinates of Goal node (x,y):")
        
        start_node = tuple(map(int, start_node_str.split(',')))
        goal_node = tuple(map(int, goal_node_str.split(',')))

        start_node = (start_node[0] // scaling_factor, start_node[1] // scaling_factor, start_node[2])
        goal_node = (goal_node[0] // scaling_factor, goal_node[1] // scaling_factor)
        

        #Check if the start and goal node are valid
        if is_valid(start_node[0],start_node[1]):

            if is_valid(goal_node[0],goal_node[1]):
                break
            else:
                print("Invalid goal node. Please enter valid coordinates.")
                continue
        else:
            print("Invalid start node. Please enter valid coordinates.")
            continue

    return start_node,goal_node

def input_clearance(key):

    while True:
        try:
            radius = int(input("Enter the " + key + " clearance (in mm): "))
            if radius < 0:
                print(key +" clearance cannot be negative. Please enter a non-negative value.")
            else:
                break           
        except ValueError:
            print("Invalid input. Please enter a valid integer for the " + key + " clearance.")

    return radius


# Input RPM1 and RPM2
def input_rpm():
    rpm_str = input("Enter the RPMs for the wheels [RPM1,RPM2]:")
    rpm = tuple(map(int, rpm_str.split(',')))

    return rpm

#checking if the robot is at a valid positon
def is_valid(x,y):

    #check if the coordinates are in bounds of canvas
    if (0 + total_clearance <= x <= width - total_clearance and 0+ total_clearance <= y <= height-total_clearance):
        pass
    else:
        return False

    # check if the coordinates are within obstacle region
    if np.array_equal(canvas[int(y), int(x)], [179, 41, 43]) or np.array_equal(canvas[int(y), int(x)], [0,0,255]):
        return False
 
    return True

def rpm2cord(actions,present_node):

    # Robot parameters
    r = 0.033*100 # wheel radius in cm 
    L = 0.287*100 # distance between wheel in cm

    # wheel velocity
    left_v = round(((2 * np.pi * actions[0]) / 60),1)
    right_v = round(((2 * np.pi * actions[1]) / 60),1)

    # Initializing Variables
    x_init = present_node[0]
    y_init = present_node[1]

    d_theta = math.radians(present_node[2])

    # d_x = 0
    # d_y =0
    
    dt = 1
    t = 0
    t_step = 1

    # distance travelled
    action_cost = 0


    d_theta = round((d_theta +((r / L) * (right_v - left_v) * dt)),1)

    d_x = 0.5*r * (left_v + right_v) * math.cos(d_theta) * dt
    d_y = 0.5*r * (left_v + right_v) * math.sin(d_theta) * dt


    action_cost = round((math.sqrt(math.pow(d_x,2)+math.pow(d_y,2))),1)
   
    # keeping theta in 0 to 2*pi range
    angle = d_theta % (2 * math.pi)

    angle = round(math.degrees(angle))

    # x & y in cm
    x = round((x_init + d_x),2)
    y = round((y_init + d_y),2)

    return x, y, angle, action_cost




def action(present_node):

    movements=[]

    action_list = [(0,rpm1),(rpm1,0),(rpm1,rpm1),(0,rpm2),(rpm2,0),(rpm2,rpm2),(rpm1,rpm2),(rpm2,rpm1)]

    # 8 actions
    for actions in action_list:
        
        x,y,angle,action_cost = rpm2cord(actions,present_node)


        if is_valid(x,y):
            movements.append(((x,y,angle),action_cost))

        else:
            movements.append((None,None))

    return movements


# Heuristic Cost 
def heuristic_cost(current_pos,goal_pos):

    cost = math.sqrt((goal_pos[1]-current_pos[1])**2+(goal_pos[0]-current_pos[0])**2) 
    return round(cost,1)

# Rounding node to nearest 0.5 multiple 
def round_node(node):

    x = round(node[0] * 2) / 2
    y = round(node[1] * 2) / 2

    # theta = round(node[2]/30)
    theta = node[2]


    return x,y,theta


#backtracking 
def get_path(start_position, goal_position,closed_list):
    
    path = []
    current_node = goal_position
    
    # Backtrack from goal node to start node
    while current_node != start_position:
        path.append(current_node)
        current_node = tuple(closed_list[current_node])
    
    # Add the start node to the path
    path.append(start_position)
    
    # Reverse the path to get it in the correct order (from start to goal)
    path.reverse()
    
    return path,closed_list


def visualization(path, closed_list, canvas, start_position, goal_position,frame_skip=450):

    #canvas cm to mm
    # resized_canvas = cv2.resize(canvas, (canvas.shape[1]*scaling_factor, canvas.shape[0]*scaling_factor))
    resized_canvas = imutils.resize(canvas, width=canvas.shape[1]*scaling_factor, height=canvas.shape[0]*scaling_factor)

    output_video = cv2.VideoWriter('astar.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 50, (resized_canvas.shape[1], resized_canvas.shape[0]))
    skip_counter = 0


    # Draw start and goal node
    cv2.circle(resized_canvas, (int(start_position[0]*scaling_factor), int(start_position[1]*scaling_factor)), 5, (0, 255, 0), -1)
    cv2.circle(resized_canvas, (int(goal_position[0]*scaling_factor), int(goal_position[1]*scaling_factor)), 5, (0, 0, 255), -1)

    for key,values in closed_list.items():

        if values is None:
            continue
        
        # child
        pt1 = (int(key[0]*scaling_factor), int(key[1]*scaling_factor))
        # parent
        pt2 = (int(values[0]*scaling_factor), int(values[1]*scaling_factor))

        cv2.arrowedLine(resized_canvas, pt2, pt1, (255,255,0), 1)

        skip_counter += 1
        if skip_counter == frame_skip:
            vid = cv2.flip(resized_canvas, 0)
            
            output_video.write(vid)
            skip_counter = 0

    # Draw the full path on the canvas
    for i in range(len(path) - 1):
        cv2.arrowedLine(resized_canvas, (int(path[i][0]*scaling_factor), int(path[i][1]*scaling_factor)), (int(path[i+1][0]*scaling_factor), int(path[i+1][1]*scaling_factor)), (0, 0, 0), thickness=2)
        vid = cv2.flip(resized_canvas, 0) 
        output_video.write(vid)

    output_video.release()


#A star algorithm
def a_star(start_position, goal_position, canvas, goal_threshold_distance= 1.5):

    # List of nodes to be explored
    open_list = []
    
    # Dictionary stores explored and its parent node
    closed_list = {}

    # Dictionary to store node information {present_node: [parent_node, cost_to_come]}
    node_info = {}

    # Closed set stores nodes that are visited (as nearest 0.5 multiple)
    closed_set = set()

    # visited  nodes (as nearest 0.5 multiple)
    visited_nodes = np.zeros(((height*2)+1,(width+1)*2), dtype=int)

    # heap to store the nodes based on their cost value
    hq.heapify(open_list)

    # Inserting the initial node with its [total_cost, present_node]
    hq.heappush(open_list, [ 0+heuristic_cost(start_position,goal_position), start_position])

    # Set the node_info for the start position
    node_info[start_position] = [None, 0]

    # visited_set.add(round_node(start_position))
    index = round_node(start_position)
    visited_nodes[int(index[1]*2)][int(index[0]*2)] = 1


    #while open list is not empty
    while open_list:

        total_cost, present_node = hq.heappop(open_list)
        # print(present_node)
        parent_node, cost2come = node_info[present_node]

        # Adding the present node to closed list with its parent node - {present_node:parent_node}

        closed_list[present_node] = parent_node

        closed_set.add(round_node(present_node))

        #Calculating cost to goal
        cost2goal = total_cost-cost2come

        #if goal distance theshold reached
        if cost2goal <= goal_threshold_distance:
            
            closed_list[goal_position] = present_node
            print("goal reached")
            return get_path(start_position, goal_position,closed_list)
    
        #Add neighbouring nodes to the open_list
        for next_node,action_cost in action(present_node):

            # next_node,action_cost = move_node(present_node,direction)

            if next_node is not None:

                rounded_next_node = round_node(next_node)

                # Calculating the index of visited nodes array
                scaled_x = int(rounded_next_node[1] * 2)
                scaled_y = int(rounded_next_node[0] * 2)
                # scaled_theta_index = int(rounded_next_node[2])

                # if all actions implemented for a particular node:
                # if np.sum(visited_nodes[scaled_x,scaled_y,:]) < 5:
                if (rounded_next_node not in closed_set):              
            
                    # print(count)
                    # if (rounded_next_node not in visited_set):
                    if (visited_nodes[scaled_x,scaled_y] == 0):

                        new_cost2come = cost2come + action_cost
                        new_total_cost = new_cost2come + heuristic_cost(next_node, goal_position)

                        hq.heappush(open_list, [new_total_cost, next_node])
                        node_info[next_node] = [present_node, new_cost2come]

                        # Set visited node as 1
                        visited_nodes[scaled_x,scaled_y] = 1
                
                    # if node is already in open list we need to compare cost and update if needed
                    else:

                        if (next_node in node_info) and (cost2come + action_cost) < node_info[next_node][1]:

                            new_cost2come = cost2come + action_cost
                            new_total_cost = new_cost2come + heuristic_cost(next_node, goal_position)
                            hq.heappush(open_list, [new_total_cost, next_node])
                            node_info[next_node] = [present_node, new_cost2come]        

    return "Solution does not exist"

                        

if __name__=="__main__":

    start_time = time.time() 

    #scaling factor
    scaling_factor = 10

    # create blank  canvas (mm to cm)
    width = 6000//scaling_factor
    height = 2000//scaling_factor
    canvas = np.ones((height,width,3), dtype=np.uint8) * 255

    # input clearance and robot radius
    robot_radius = 220
    robot_clearance = input_clearance("robot")
    obstacle_clearance = input_clearance("obstacle")

    total_clearance =  (robot_clearance + obstacle_clearance + robot_radius)//scaling_factor

    # draw the obstacle map
    canvas = obstacle_map(canvas)

    # input start and goal node coordinates
    start_position,goal_position = input_coordinates()

    #input RPMs
    rpm1,rpm2 = input_rpm()

    # A*
    path,closed_list = a_star(start_position, goal_position, canvas)

    #goal reached
    goal_reached_time = time.time()
    print("Total time taken to reach the goal: ",goal_reached_time-start_time)

    # Display Node exploration and Optimal path
    visualization(path,closed_list,canvas,start_position,goal_position)

    end_time = time.time()

    print("Total time taken to execute the code: ",end_time-start_time) 
    # print(f"closed_list : {closed_list}")
    # print(f"path : {path}")
    