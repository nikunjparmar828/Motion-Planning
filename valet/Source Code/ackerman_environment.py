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
        self.margin = 10 #not in use at the moment
        self.car_length = 80 #not the wheel base 
        self.car_width = 40
        self.l = 50 # wheel base 

        self.grd[100,100] = 5
        self.grd[629, 289] = 6 # location parallel to the next car in the parking lot

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
        
        # adding car body
        rotated_struct = self.rotate_car(self.car_struct, angle=psi)
        rotated_struct += np.array([x,y]) # 4 points of car --> draw line between them

        temp_env = grd1.copy()   
        for x,y in rotated_struct:
            
            temp_env[x][y] = 8
        
        return temp_env, rotated_struct
        
    
    def ackerman_kine(self, grd2, start, end):
        cntr = 0

        l = 50 # wheel span 
        dt = 4 #time stamp

        u_phi = [math.radians(-45),
                 math.radians(-30),
                 math.radians(-20),
                 0,
                 math.radians(20),
                 math.radians(30),
                 math.radians(45)]
        
        u_s = [-1,-0.5,-0.1,0.1,0.5,1] # change accordingly
    
        end_new = [] # tracks the end position of the path before parking 

        prev = []                                                       # keeps the track of previous nodes
        grd_trc = [[-1 for col in range(800)] for row in range(800)]    # keeps the track of sequence of previously visited node
        dist_from_start = [[math.inf for col in range(800)] for row in range(800)] #inf matrix of 800*800

        current = start                         #represents the current node which is the starting point in our case 
        start_angle = 1.5708
        # start_angle = 0

        seen = []                               #tracks the visited points
        heap = []                               # here we are using heap instead of queue to optimize the time complexity of the program
        
        x = current[0]                          #x,y represent the starting coordinates
        y = current[1]

        dist_from_start[x][y] = 0               #distance for the starting point = 0
        # dist_from_start represents the distance from start point to the next point
        
        seen.append(current)                    #adding the visited point into the seen list
        
        found = False
        resign = False
        
        heapq.heappush(heap, [dist_from_start[x][y], [x, y], start_angle]) 
                
        while found == False and resign == False:
            
            if len(heap) == 0:              #there is no path if heap is empty
                # resign = True
                print("No path for you! sad!")
                break
            
            else:
                node = heapq.heappop(heap)      #pop and return the smallest value from the heap
                
                a = node[0]
                x = node[1][0]
                y = node[1][1]

                start_angle = node[2]
                prev.append([x,y])
    
                end_new = [x,y]

                if (grd2[x][y]==6):                  #Remeber, 6 is the value of the end position in our 2D grid
                    found = True
                    print("Path found")
                    break
                else:
                   
                    for U_phi in u_phi:
                        for U_s in u_s:

                            theta_new = start_angle + ((math.tan(U_phi)*U_s)/l)*dt 

                            x1 = x + U_s*math.cos(theta_new)*dt
                            y1 = y - U_s*math.sin(theta_new)*dt
                            
                            x1_int, y1_int = (int(x1), int(y1))

                            if 0 <= x1 and x1< 799 and 0 <= y1 and y1< 799 and grd2[x1_int][y1_int] != 1:
                                cost = math.sqrt((x1-x)**2 +(y1-y)**2) #measures the distance between two neighbours
                                if(dist_from_start[x_int][y_int] + cost < dist_from_start[x1_int][y1_int]): #update the cost if new cost is greater than the previous cost
                                    
                                    dist_from_start[x1_int][y1_int] = dist_from_start[x_int][y_int] + cost
                                    heapq.heappush(heap, [dist_from_start[x1_int][y1_int], [x1, y1], theta_new])
                                    grd_trc[x1_int][y1_int] = [[x,y],theta_new]               

        #Back tracing of the path-------------------------------------------------------------     
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
                    
                    x2_ = int(gd[1][0])         # Back tracing the path.
                    y2_ = int(gd[1][1])

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

