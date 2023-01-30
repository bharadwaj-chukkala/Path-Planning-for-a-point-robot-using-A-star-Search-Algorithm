#IMPORTING NECESSARY LIBRARIES
import numpy as np
from math import dist
import matplotlib.pyplot as plt
import time
import heapq

########## DEFINING A NODE CLASS TO STORE NODES AS OBJECTS ###############

class Node:

    def __init__(self, x, y, theta, cost, parent_id, c2g = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.parent_id = parent_id
        self.c2g = c2g 
        
        
    def __lt__(self,other):
        return self.cost + self.c2g < other.cost + other.c2g

########### DEFINING ACTIONS TO BE PERFORMED ##############
########### CALCULATING COST TO COME FOR ALL ACTIONS ########

def move_60up(x,y,theta,step_size, cost):
    theta = theta + 60
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    #x = round(x*2)/2
    #y = round(y*2)/2
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta,cost

def move_30up(x,y,theta, step_size, cost):
    theta = theta + 60
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    #x = round(x*2)/2
    #y = round(y*2)/2
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta, cost

def move_0(x,y,theta, step_size, cost):
    theta = theta + 0
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    #x = round(x*2)/2
    #y = round(y*2)/2
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta, cost

def move_30down(x,y,theta, step_size, cost):
    theta = theta - 30
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    #x = round(x*2)/2
    #y = round(y*2)/2
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta, cost

def move_60down(x,y,theta, step_size, cost):
    theta = theta - 60
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    #x = round(x*2)/2
    #y = round(y*2)/2
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta,cost



############ DEFINING A FUNCTION TO PERFORM ACTIONS THAT ARE DEFINED #########

def Action_set(move,x,y,theta,step_size,cost):

	if move == 'ext_right':
		return move_60up(x,y,theta, step_size,cost)
	elif move == 'right':
		return move_30up(x,y,theta, step_size,cost)
	elif move == 'straight':
		return move_0(x,y,theta,step_size,cost)
	elif move == 'left':
		return move_30down(x,y,theta,step_size,cost)
	elif move == 'ext_left':
		return move_60down(x,y,theta,step_size,cost)
	else:
		return None

############ CONFIGURATION SPACE CONSTRUCTION WITH OBSTACLES ############

def C_obs_space(width, height, obs_clearance, robot_radius):

    obs_space = np.full((height,width),0)
    
    circle_radius = 40
    
    for y in range(height) :
        for x in range(width):
            
        ####### CLEARANCE FOR THE OBSTACLES #######
            
            #Polygon Obstacle (Clearance)
            t1 = (y- ( obs_clearance + robot_radius)) - ((0.316) *(x + ( obs_clearance + robot_radius))) - 173.608
            t2 = (y+ ( obs_clearance + robot_radius)) + (1.23 * (x + ( obs_clearance + robot_radius))) - 229.34 
            t3 = (y- ( obs_clearance + robot_radius)) + (3.2 * (x- ( obs_clearance + robot_radius))) - 436 
            t4 = (y+ ( obs_clearance + robot_radius)) - 0.857*(x- ( obs_clearance + robot_radius)) - 111.42 
            t5 = y + (0.1136*x) - 189.09
            
            #Circle Obstacle (Clearance)
            C = ((y -185)**2) + ((x-300)**2) - (circle_radius + obs_clearance + robot_radius)**2  
            
            #Hexagon Obstacle (Clearance)
            h1 = (y- ( obs_clearance + robot_radius)) - 0.577*(x+ ( obs_clearance + robot_radius)) - 24.97
            h2 = (y- ( obs_clearance + robot_radius)) + 0.577*(x- ( obs_clearance + robot_radius)) - 255.82
            h3 = (x- ( obs_clearance + robot_radius)) - 235 
            h6 = (x+ ( obs_clearance + robot_radius)) - 165 
            h5 = (y+ ( obs_clearance + robot_radius)) + 0.577*(x+ ( obs_clearance + robot_radius)) - 175 
            h4 = (y+ ( obs_clearance + robot_radius)) - 0.577*(x- ( obs_clearance + robot_radius)) + 55.82 
            
            #Conditions defining all points bounded by these lines are in the obstacle clearance area
            if(h1<0 and h2<0 and h3<0 and h4>0 and h5>0 and h6>0) or C<=0  or (t1<0 and t5>0 and t4>0)or (t2>0 and t5<0 and t3<0):
                obs_space[y,x] = 1
               
             
        ########  OBSTACLES   ########
            
            #Hexagon Obstacle    
            a1 = y - 0.577*x - 24.97 
            a2 = y + 0.577*x - 255.82
            a3 = x - 235 
            a6 = x - 165 
            a5 = y + 0.577*x - 175 
            a4 = y - 0.577*x + 55.82 
            
            #Circle Obstacle
            D = ((y -185)**2) + ((x-300)**2) - (circle_radius)**2 
            
            #Polygon Obstacle
            l1 = y - ((0.316) *x) - 173.608  
            l2 = y + (1.23 * x) - 229.34 
            l3 = y + (3.2 * x) - 436 
            l4 = y - 0.857*x - 111.42 
            l5 = y + (0.1136*x) - 189.09
            
            #Conditions defining all points bounded by these lines are in the obstacle area
            if(a1<0 and a2<0 and a3<0 and a4>0 and a5>0 and a6>0) or D<0 or (l1<0 and l5>0 and l4>0)or (l2>0 and l5<0 and l3<0):
                obs_space[y,x] = 2
                
                
####### DEFINING THE BOUNDARIES FOR CONFIGURATION SPACE ########

    for i in range(400):
        obs_space[0][i] = 1
        
    for i in range(400):
        obs_space[249][i] = 1
        
    for i in range(250):
        obs_space[i][1] = 1
        
    for i in range(250):
        obs_space[i][399] = 1
       
    return obs_space

########## TO SEE IF THE MOVE IS VALID OR NOT #########

def ValidMove(x, y, obs_space):

	e = obs_space.shape

	if( x > e[1] or x < 0 or y > e[0] or y < 0 ):
		return False
	
	else:
		try:
			if(obs_space[y][x] == 1  or obs_space[y][x]==2):
				return False
		except:
			pass
	return True

########## DEFINING A FUNCTION TO CHECK IF THE PRESENT NODE IS GOAL NODE ##########

def Check_goal(present, goal):
    
    dt = dist((present.x, present.y), (goal.x, goal.y))             

    if dt < 1.5:
        return True
    else:
        return False

######   TO SEE IF THE OREINTATION IS VALID OR NOT  ######

def validorient(theta):
    if((theta%30)==0):
        return theta
    else:
        return False
    

######### GENERATE UNIQUE KEY ##########

def key(node):
    key = 1022*node.x + 111*node.y 
    return key

########## A STAR ALGORITHM ###########

def a_star(start,goal,obs_space,step_size):                       

    if Check_goal(start, goal):
        return None,1
    goal_node = goal
    start_node = start
    
    moves = ['ext_right','right', 'straight', 'left', 'ext_left']   
    unexplored_nodes = {}  #List of all open nodes
    
    start_key = key(start_node) #Generating a unique key for identifying the node
    unexplored_nodes[(start_key)] = start_node
    
    explored_nodes = {} #List of all closed nodes
    priority_list = []  #List to store all dictionary entries with cost as the sorting variable
    heapq.heappush(priority_list, [start_node.cost, start_node]) #This Data structure will prioritize the node to be explored which has less cost.
    
    all_nodes = [] #stores all nodes that have been traversed, for visualization purposes.
    

    while (len(priority_list) != 0):

        present_node = (heapq.heappop(priority_list))[1]
        all_nodes.append([present_node.x, present_node.y, present_node.theta])          
        present_id = key(present_node)
        if Check_goal(present_node, goal_node):
            goal_node.parent_id = present_node.parent_id
            goal_node.cost = present_node.cost
            print("Goal Node found")
            return all_nodes,1

        if present_id in explored_nodes:  
            continue
        else:
            explored_nodes[present_id] = present_node
		
        del unexplored_nodes[present_id]

        for move in moves:
            x,y,theta,cost = Action_set(move,present_node.x,present_node.y,present_node.theta, step_size, present_node.cost)  ##newaddd
            
            c2g = dist((x, y), (goal.x, goal.y))  
   
            new_node = Node(x,y,theta, cost,present_node, c2g)   
   
            new_node_id = key(new_node) 
   
            if not ValidMove(new_node.x, new_node.y, obs_space):
                continue
            elif new_node_id in explored_nodes:
                continue
   
            if new_node_id in unexplored_nodes:
                if new_node.cost < unexplored_nodes[new_node_id].cost: 
                    unexplored_nodes[new_node_id].cost = new_node.cost
                    unexplored_nodes[new_node_id].parent_id = new_node.parent_id
            else:
                unexplored_nodes[new_node_id] = new_node
   			
            heapq.heappush(priority_list, [(new_node.cost + new_node.c2g), new_node]) 
   
    return  all_nodes,0

########### BACKTRACK AND GENERATE SHORTEST PATH ############

def Backtrack(goal_node):  
    x_path = []
    y_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)

    parent_node = goal_node.parent_id
    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        parent_node = parent_node.parent_id
        
    x_path.reverse()
    y_path.reverse()
    
    x = np.asarray(x_path)
    y = np.asanyarray(y_path)
    
    return x,y

    

#########  PLOT OBSTACLES SPACE, EXPLORED NODES, SHORTEST PATH  #######

def plot(start_node,goal_node,x_path,y_path,all_nodes,obs_space):
    plt.figure()
    ### Start node and Goal node ###
    plt.plot(start_node.x, start_node.y, "Dw")
    plt.plot(goal_node.x, goal_node.y, "Dg")

    ### Configuration Space for Obstacles ####
    plt.imshow(obs_space, "GnBu")
    ax = plt.gca()
    ax.invert_yaxis() #y-axis inversion
    
    ### All visited nodes ###
    for i in range(len(all_nodes)):
        plt.plot(all_nodes[i][0], all_nodes[i][1], "2g-")
        #plt.pause(0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)

    ### Shortest path found ###
    plt.plot(x_path,y_path, ':r')
    #for i in range(len(x_path)):
    #plt.quiver(x_path[0], y_path[0], x_path[len(x_path)-1], y_path[len(y_path)-1], scale_units='xy',angles= 'xy' ,scale=0.1)
    #plt.quiver(x_path, y_path, x_path-1, y_path-1, scale_units='xy',scale_angles= 'xy' ,scale=1)
    #plt.quiver(x_path[:-1], y_path[:-1], x_path[1:]-x_path[:-1], y_path[1:]-y_path[:-1], units='xy', scale=1)#, minshaft= 2)#, , angles='xy', width=.01, headlength=4, headwidth=4, minshaft= 2)
    plt.show()
    plt.pause(3)
    plt.close('all')




######### CALLING ALL MY FUNCTIONS TO IMPLEMENT A STAR ALGORITHM ON A POINT ROBOT ###########

if __name__ == '__main__':
    
    #### Clearance of the Obstacle ####
    obs_clearance = input("Assign Clearance to the Obstacles: ")
    obs_clearance = int(obs_clearance)
    
    #### Radius of the Robot ####
    robot_radius = input("Enter the Radius of the Robot: ") 
    robot_radius = int(robot_radius)
    
    #### Step Size of the Robot ####
    robot_step_size = input("Enter Step size of the Robot: ")
    robot_step_size = int(robot_step_size)
    
    width = 400
    height = 250
    obs_space = C_obs_space(width, height, obs_clearance, robot_radius)
    c2g = 0
    
    #### Taking start node coordinates as input from user #####
    start_coordinates = input("Enter coordinates for Start Node: ")
    s_x, s_y = start_coordinates.split()
    s_x = int(s_x)
    s_y = int(s_y)
    
    #### Taking Orientation for the robot ####
    s_theta = input("Enter Orientation of the robot at start node: ")
    s_t = int(s_theta)
    
    ### Checking if the user input is valid #####
    if not ValidMove(s_x, s_y, obs_space):
        print("Start node is out of bounds")
        exit(-1)
        
    if not validorient(s_t):
        print("Orientation has to be a multiple of 30")
        exit(-1)
		    
	##### Taking Goal node coordinates as input from user ##### 
    goal_coordinates = input("Enter coordinates for Goal Node: ")
    g_x, g_y = goal_coordinates.split()
    g_x = int(g_x)
    g_y = int(g_y)
    
    #### Taking Orientation for the robot ####
    g_theta = input("Enter Orientation of the robot at goal node: ")
    g_t = int(g_theta)
    
    
    ### Checking if the user input is valid #####
    if not ValidMove(g_x, g_y, obs_space):
        print("Goal node is out of bounds")
        exit(-1)
        
    if not validorient(g_t):
        print("Orientation has to be a multiple of 30")
        exit(-1)

    ### Timer to calculate computational  time ###
    timer_start = time.time()
    
	##### Creating start_node and goal_node objects 
    start_node = Node(s_x, s_y,s_t, 0.0, -1,c2g)
    goal_node = Node(g_x, g_y,g_t, 0.0, -1, c2g)
    all_nodes,flag = a_star(start_node, goal_node, obs_space, robot_step_size)
    
    ##### Plot shortest path only when goal node is reached #####
    if (flag)==1:
        x_path,y_path = Backtrack(goal_node)
    else:
        print("No path was found")
		
    plot(start_node,goal_node,x_path,y_path,all_nodes,obs_space)
    timer_stop = time.time()
    
    C_time = timer_stop - timer_start
    print("The Total Runtime is:  ", C_time) 
	



	











