import numpy as np
import matplotlib.pyplot as plt
import random
import collections
import cv2
import math
import heapq

class Environment1:

    def __init__(self):
        self.grd = np.zeros((250,250))  
        # self.start = [10,10] 

        #don't forget to change the value on astar planner
        self.grd[10,10] = 100 #starting point has value 100 assigned to it in the matrix

        ## bush_state = 0.1 --> Burned 
        ## bush_state = 0.5 --> Extenguished
        ## bush_state = 3 --> Burning

        self.bush_state = 0.5 #not on fire
        # self.bush_red = 95 #on fire

        self.bush_generated = [] #counts number of bush and its state [[bush_number, state_of_bush]]

        self.car_length = 5
        self.car_width = 2

        self.car_struct = np.array([[+self.car_length/2, +self.car_width/2],
                                    [+self.car_length/2, -self.car_width/2],  
                                    [-self.car_length/2, -self.car_width/2],
                                    [-self.car_length/2, +self.car_width/2]], 
                                    np.int32)
#---------------------------------------------------------------------------------------------------------------------        
    def rotate_car(self, pts, angle):
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        return ((R @ pts.T).T).astype(int)

#-------------------------------------------------------------------------------------------------------------------
    def render(self, grd1, x, y, psi):
        
        x = int(x)
        y = int(y)
        
        # adding vehicle's body
        rotated_struct = self.rotate_car(self.car_struct, angle=psi)
        rotated_struct += np.array([x,y]) # 4 points of the vehicle

        temp_env = grd1

        #drawing the vehicle --> assign value 8 to all four corner of the vehicle
        
        for x,y in rotated_struct:  
            temp_env[x][y] = 8
               
        return temp_env, rotated_struct
#-------------------------------------------------------------------------------------------------------------------
    
    def bush_gen(self,density):

        total_cells = 250*250
        total_bushes = int(round( (total_cells * (density/100))/225))
       

        #As we know we have four different obstacles in a set so we require 
        for p in range(total_bushes):

            rand_no_bush_i = random.randint(10,240)
            rand_no_bush_j = random.randint(10,240)

            # rand_no_bush_i, rand_no_bush_j --> Center point of bushes 
            # 'int' --> intact
            # 'bur' --> on fire
            # 'ext' --> extinguished 

            self.bush_generated.append([[rand_no_bush_i, rand_no_bush_j], 'int'])

            #local square grid generation 

            i = random.randint(rand_no_bush_i-7, rand_no_bush_i+7)                               #it generates a random intiger each time
            j = random.randint(rand_no_bush_j-7, rand_no_bush_j+7)

            bush_congestion = 10 # defines the congestion of tetrominoes inside the 15*15 square metere obstacle

            for num_sub_tetro in range(bush_congestion):

                k = random.randint(0,4)

                if k==0:
                    inner_counter = 0 
                    while True:
                        inner_counter+=1
                        if(i<rand_no_bush_i+7-4):
                            if(self.grd[i,j]==0 and self.grd[i+1,j]==0 and self.grd[i+2,j]==0 and self.grd[i+3,j]==0):
                        
                                self.grd[i,j] = self.bush_state
                                self.grd[i+1,j] = self.bush_state
                                self.grd[i+2,j] = self.bush_state
                                self.grd[i+3,j] = self.bush_state
                                break
                        if(inner_counter==200):
                            break 
                        i = random.randint(rand_no_bush_i-7+1,rand_no_bush_i+7-2)
                        
                elif k==1:
                    inner_counter=0
                    while True:
                        inner_counter+=1
                        if(i<rand_no_bush_i+7-3 and j<rand_no_bush_j+7-2):
                            if(self.grd[i,j]==0 and self.grd[i+1,j]==0 and self.grd[i+1,j+1]==0 and self.grd[i+2,j+1]==0):
                                self.grd[i,j] = self.bush_state
                                self.grd[i+1,j] = self.bush_state
                                self.grd[i+1,j+1] = self.bush_state
                                self.grd[i+2,j+1] = self.bush_state
                                break
                        
                        if(inner_counter==200):
                            break

                        i = random.randint(rand_no_bush_i-7+1,rand_no_bush_i+7-2)                        #it generates a random intiger each time
                        j = random.randint(rand_no_bush_j-7+1,rand_no_bush_j+7-2)

                elif k==2:
                    inner_counter=0
                    while True:
                        inner_counter+=1
                        if(i<rand_no_bush_i+7-3 and j<rand_no_bush_j+7-2):
                            if(self.grd[i,j]==0 and self.grd[i,j+1]==0 and self.grd[i+1,j+1]==0 and self.grd[i+2,j+1]==0):
                                self.grd[i,j] = self.bush_state
                                self.grd[i,j+1] = self.bush_state
                                self.grd[i+1,j+1] = self.bush_state
                                self.grd[i+2,j+1] = self.bush_state
                                break

                        if(inner_counter==200):
                            break
                        i = random.randint(rand_no_bush_i-7+1,rand_no_bush_i+7-2)                       #it generates a random intiger each time
                        j = random.randint(rand_no_bush_j-7+1,rand_no_bush_j+7-2)
                        
                elif k==3:
                    inner_counter=0
                    while True:
                        inner_counter+=1
                        if(i<rand_no_bush_i+7-2 and j<rand_no_bush_j+7-2):
                            if(self.grd[i,j]==0 and self.grd[i,j+1]==0 and self.grd[i-1,j+1]==0 and self.grd[i+1,j+1]==0):
                                self.grd[i,j] = self.bush_state
                                self.grd[i,j+1] = self.bush_state
                                self.grd[i-1,j+1] = self.bush_state
                                self.grd[i+1,j+1] = self.bush_state
                                break
                        if(inner_counter==200):
                            break
                        
                        i = random.randint(rand_no_bush_i-7+1,rand_no_bush_i+7-2)                       #it generates a random intiger each time
                        j = random.randint(rand_no_bush_j-7+1,rand_no_bush_j+7-2)
        
        return self.grd, self.bush_generated

    def rand_fire_at_60_sec(self,updated_env, bush_list_with_state):
        
        xbush, ybush = 0,0

        for counter in range(len(bush_list_with_state)):
            rand_num = random.randint(0,len(bush_list_with_state)-1)
            bush_pos = bush_list_with_state[rand_num]

            if(bush_pos[1]=='int'): # if the random bush is intact than set it to fire
                xbush, ybush = bush_pos[0]
                bush_list_with_state[rand_num][1] = 'bur'
                bush_at_fire = bush_list_with_state[rand_num]
                break

        for k in range(xbush-9, xbush+9):
            for l in range(ybush-9, ybush+9):
                if(k>0 and l>0 and k<249 and l<249):
                    if (updated_env[k][l]==0.5):
                        updated_env[k][l] = 3               

        return bush_list_with_state, updated_env, bush_at_fire

    def neighbour_fire_at_20_sec(self, updated_env, bush_list_with_state):

        child_bushes_at_fire = []

        temp_bush_list_with_state = bush_list_with_state

        for parent_bush_no in range(0, len(bush_list_with_state)):

            if (bush_list_with_state[parent_bush_no][1] == 'bur'):
                xbur, ybur = bush_list_with_state[parent_bush_no][0]

                for iter in range(0, len(bush_list_with_state)):
                    if (bush_list_with_state[iter][1] == 'int'):
                        xchild, ychild = bush_list_with_state[iter][0]
                        dist_30 = math.sqrt( (xbur - xchild)**2 + (ybur-ychild)**2)

                        if (dist_30<=30):
                            temp_bush_list_with_state[iter][1] = 'bur'
                            child_bushes_at_fire.append(temp_bush_list_with_state[iter])
                               
        bush_list_with_state = temp_bush_list_with_state # reassigning the values 
                        
        # updating the environment for burning childs 

        for cntr in range(0, len(bush_list_with_state)):
            if bush_list_with_state[cntr][1] == 'bur':
                xbush, ybush = bush_list_with_state[cntr][0]
                for k in range(xbush-9, xbush+9):
                    for l in range(ybush-9, ybush+9):
                        if(k>0 and l>0 and k<249 and l<249):
                            if (updated_env[k][l]==0.5):
                                updated_env[k][l] = 3
                
        return bush_list_with_state, updated_env, child_bushes_at_fire

    def end_state_update(self, bush_at_fire, updated_env, bush_list_with_state):

        if bush_at_fire[1] == 'bur':
            num_num = bush_list_with_state.index(bush_at_fire)
            bush_list_with_state[num_num][1] = 'ext'

            xbush, ybush = bush_at_fire[0]
            
            for k in range(xbush-9, xbush+9):
                for l in range(ybush-9, ybush+9):
                    if(k>0 and l>0 and k<249 and l<249):
                        if (updated_env[k][l]==3):
                            updated_env[k][l] = 0.1 # setting bush to extinguished value
        
        return bush_list_with_state, updated_env
#-------------------------------------------------------------------------------------------------------------------
## astar_planner --> class
#-----------------------------------------------------------------------------------------------------------------------
class prm_planner:
    
    def __init__(self):

        self.car_length = 5
        self.car_width = 2

    def inflated_env(self, grd3, bush_list_with_state):

        # boundary generation 
        for i in range(250):
            grd3[i][0] = 1
            grd3[i][1] = 1
            grd3[i][2] = 1
            grd3[i][3] = 1
            grd3[i][4] = 1

            grd3[0][i] = 1
            grd3[1][i] = 1
            grd3[2][i] = 1
            grd3[3][i] = 1
            grd3[4][i] = 1

            grd3[i][249] = 1
            grd3[i][248] = 1
            grd3[i][247] = 1
            grd3[i][246] = 1
            grd3[i][245] = 1
            grd3[i][244] = 1

            grd3[249][i] = 1
            grd3[248][i] = 1
            grd3[247][i] = 1
            grd3[246][i] = 1
            grd3[245][i] = 1

        for j in range(len(bush_list_with_state)):

            xc,yc = bush_list_with_state[j][0]

            for k in range(xc-9, xc+9):
                for l in range(yc-9, yc+9):
                    if(k>0 and l>0 and k<249 and l<249):
                        grd3[k][l] = 1

        return grd3


    def path_generation(self, grd2, start, end):

        # print(end)
        x_goal = int(end[0])
        y_goal = int(end[1])

        x_start = start[0]
        y_start = start[1]

        grd2[x_goal][y_goal] = 6 # As you know goal point is defined as values 6 
        grd2[x_start][y_start] = 100

        r=2 #wheel radius
        l=20 #wheel span 
        dt = 5 #time stamp
        ur = [-1,0,1]
        ul = [-1,0,1]
        motion_conn = [[-1,-1],
                        [-1,0],
                        [-1,1],
                        [0,-1],
                        [0,0],
                        [0,1],
                        [1,1],
                        [1,0],
                        [1,1]]
        
        end_new = []
        prev = []    
        seen = []                               #tracks the visited points
        heap = []                               # here we are using heap instead of queue to optimize the time complexity of the program
                                                        # keeps the track of previous nodes
        grd_trc = [[-1 for col in range(250)] for row in range(250)]    # keeps the track of sequence of previously visited node
        dist_from_start = [[math.inf for col in range(250)] for row in range(250)] #inf matrix of 800*800

        current = start                         #represents the current node which is the starting point in our case 
        start_angle = 0
   
        x = current[0]                          #x,y represent the starting coordinates
        y = current[1]

        dist_from_start[x][y] = 0               #distance for the starting point = 0
        # dist_from_start represents the distance from start point to the next point
        
        seen.append(current)                    #adding the visited point into the seen list
        
        found = False
        resign = False
        
        heapq.heappush(heap, (dist_from_start[x][y], (x, y), start_angle)) 
                
        while found == False and resign == False:
            
            if len(heap) == 0:              #there is no path if heap is empty
                # resign = True
                print("No path for you! sad!")
                break
            
            else:
                node = heapq.heappop(heap)      #pop and return the smallest value from the heap
                
                a = node[0]
                x = node[1][0]
                y = node [1][1]
                start_angle = node[2]
                
                prev.append([x,y])
                
                end_new = [x,y]

                dist_rad = math.sqrt ( (x_goal-x)**2 + (y_goal-y)**2 )

                if (dist_rad<=11):                  #Remeber, 6 is the value of the end position in our 2D grid
                    found = True
                    print("Path found")
                    break
                else:     
                    for ii,item in enumerate(motion_conn): 
                        # print([ii, type(ii), type(item)])
                        theta_new = start_angle + ((r/l)*(item[0] - item[1]) )*dt 

                        x1 = int(x + (r/2)*(item[0] + item[1])*math.cos(theta_new)*dt)

                        y1 = int(y + (r/2)*(item[0] + item[1])*math.sin(theta_new)*dt)

                        #collision check and path generation
                        if 0 <= x1 and x1< 249 and 0 <= 249 and y1< 249 and grd2[x1][y1] != 1:
                            cost = math.sqrt((x1-x)**2 +(y1-y)**2) + math.sqrt((x_goal-x1)**2 +(y_goal-y1)**2)#measures the distance between two neighbours
                            
                            if(dist_from_start[x][y] + cost < dist_from_start[x1][y1]): #update the cost if new cost is greater than the previous cost
                                
                                dist_from_start[x1][y1] = dist_from_start[x][y] + cost
                                heapq.heappush(heap, (dist_from_start[x1][y1], (x1, y1), theta_new))
                                grd_trc[x1][y1] = [ii,[x,y],theta_new]

        ## back Tracking of the path------------------------------------------------------------------   
        if resign == False:

            counter = 0
            new_path = []
            angle_list = []

            new_path.append(end_new)
            
            x_ = end_new[0]    
            y_ = end_new[1]

            while ( int(x_) != x_start ) or (int(y_) != y_start):
                if counter < 2000:
                    
                    gd = grd_trc[x_][y_] 
                    
                    x2_ = gd[1][0]         # Back tracing the path.
                    y2_ = gd[1][1]

                    new_path.append([x2_, y2_])
                    angle_list.append(gd[2])
                    counter+=1

                    x_ = x2_
                    y_ = y2_
                else:
                    break
                
            new_path.reverse() # The path in the final_path will be from end to start to we will reverse it.
            angle_list.reverse()
            return new_path, angle_list
