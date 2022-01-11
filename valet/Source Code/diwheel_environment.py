import numpy as np
import matplotlib.pyplot as plt
import random
import collections
import math
import heapq
import cv2

class Environment():

    def __init__(self):
        
        self.grd = np.zeros((800,800)) #creates 800*800 zero value array or a 2D grid with each cell's value is 0 
        self.margin = 10

        self.car_length = 20
        self.car_width = 40
        
        self.grd[100,100] = 5 # Start point is assigned value 5
        self.grd[729, 194] = 6 # parking location is assigned values 6 

        # self.wheel_positions = np.array([[25,15],[25,-15],[-25,15],[-25,-15]])       

        self.car_struct = np.array([[+self.car_length/2, +self.car_width/2],
                                    [+self.car_length/2, -self.car_width/2],  
                                    [-self.car_length/2, -self.car_width/2],
                                    [-self.car_length/2, +self.car_width/2]], 
                                    np.int32)

    def obs_gen(self):

        # boundary generation 
        for i in range(800):
            self.grd[i][0] = 1
            self.grd[0][i] = 1
            self.grd[i][799] = 1
            self.grd[799][i] = 1

        #obstacle(s) generation 
        for j in range(200):
            self.grd[400][350+j] = 1
            self.grd[600][350+j] = 1
            self.grd[400+j][350] = 1
            self.grd[400+j][550] = 1

        #car obstacles
        car_length = 80
        car_width = 40

        for m in range(2):
            if (m==0):
                for k in range(car_length):
                    self.grd[749][50+k] = 1
                    self.grd[709][50+k] = 1
                for l in range(car_width):
                    self.grd[709+l][50] = 1
                    self.grd[709+l][129] = 1
            if (m==1):
                for k in range(car_length):
                    self.grd[749][280+k] = 1
                    self.grd[709][280+k] = 1
                for l in range(car_width):
                    self.grd[709+l][280] = 1
                    self.grd[709+l][360] = 1

        
        return self.grd
    
    def rotate_car(self, pts, angle):
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        return ((R @ pts.T).T).astype(int)

    def render(self, grd1, x, y, psi):  
        x = int(x)
        y = int(y)
        
        # adding vehicle's body
        rotated_struct = self.rotate_car(self.car_struct, angle=psi)
        rotated_struct += np.array([x,y]) # 4 points of the vehicle

        temp_env = grd1.copy()

        #drawing the vehicle --> assign value 8 to all four corner of the vehicle
        
        for x,y in rotated_struct:  
            temp_env[x][y] = 8
        return temp_env, rotated_struct
    ##-------------------------------------------------------------------------------------------------------------------
    
    def diwheel_kine(self, grd2, start, end):
        
        r=4 #wheel radius
        l=40 #wheel span 
        dt = 2 #time stamp
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
        prev = []                                                       # keeps the track of previous nodes
        grd_trc = [[-1 for col in range(800)] for row in range(800)]    # keeps the track of sequence of previously visited node
        dist_from_start = [[math.inf for col in range(800)] for row in range(800)] #inf matrix of 800*800

        current = start                         #represents the current node which is the starting point in our case 
        start_angle = 0

        seen = []                               #tracks the visited points
        heap = []                               # here we are using heap instead of queue to optimize the time complexity of the program
        
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
                resign = True
                print("No path for you! sad!")
            
            else:
                node = heapq.heappop(heap)      #pop and return the smallest value from the heap
                
                a = node[0]
                x = node[1][0]
                y = node [1][1]
                start_angle = node[2]
                
                prev.append([x,y])
                
                end_new = [x,y]
                if (grd2[x][y]==6):                  #Remeber, 6 is the value of the end position in our 2D grid
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
                        if 0 <= x1 and x1< 799 and 0 <= y1 and y1< 799 and grd2[x1][y1] != 1:
                            cost = math.sqrt((x1-x)**2 +(y1-y)**2) #measures the distance between two neighbours
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

            while ( int(x_) != 100 ) or (int(y_) != 100):
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

## Code to generate lines between all the corners of the vehicle----------------------------------

    # def draw_line(mat, x0, y0, x1, y1, inplace=False):
    #     # if not (0 <= x0 < 800 and 0 <= x1 < 800 and
    #     #         0 <= y0 < 800 and 0 <= y1 < 800):
    #     #     raise ValueError('Invalid coordinates.')
    #     # if not inplace:
    #     #     mat = mat.copy()
    #     # if (x0, y0) == (x1, y1):
    #     #     mat[x0, y0] = 2
    #     #     return mat if not inplace else None
    #     # Swap axes if Y slope is smaller than X slope
    #     transpose = abs(x1 - x0) < abs(y1 - y0)
    #     if transpose:
    #         mat = mat.all().T
    #         x0, y0, x1, y1 = y0, x0, y1, x1
    #     # Swap line direction to go left-to-right if necessary
    #     if x0 > x1:
    #         x0, y0, x1, y1 = x1, y1, x0, y0
    #     # Write line ends
    #     mat[x0, y0] = 2
    #     mat[x1, y1] = 2
    #     # Compute intermediate coordinates using line equation
    #     x = np.arange(x0 + 1, x1)
    #     y = np.round(((y1 - y0) / (x1 - x0)) * (x - x0) + y0).astype(x.dtype)
    #     # Write intermediate coordinates
    #     mat[x, y] = 3
    #     if not inplace:
    #         return mat if not transpose else mat.all().T
