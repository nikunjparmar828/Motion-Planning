import numpy as np
import matplotlib.pyplot as plt
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

        self.trailer_length = 20 #not the wheel base 
        self.trailer_width = 40
        
        self.grd[150,150] = 5
        self.grd[629, 289] = 6 # location parallel to the next car in the parking lot

        # self.wheel_positions = np.array([[25,15],[25,-15],[-25,15],[-25,-15]])       

        self.car_struct = np.array([[+65, +self.car_width/2],
                                    [+65, -self.car_width/2],  
                                    [-15, -self.car_width/2],
                                    [-15, +self.car_width/2]], 
                                    np.int32)

        self.trailer_struct = np.array([[+self.trailer_length/2, +self.trailer_width/2],
                                    [+self.trailer_length/2, -self.trailer_width/2],  
                                    [-self.trailer_length/2, -self.trailer_width/2],
                                    [-self.trailer_length/2, +self.trailer_width/2]], 
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

    def render(self, grd1, x_car, y_car,x_trailer,y_trailer, psi_car, psi_trailer):
        
        x_car = int(x_car)
        y_car = int(y_car)

        x_trailer = int(x_trailer)
        y_trailer = int(y_trailer)
        
        # adding car body to the environment
        rotated_struct_car = self.rotate_car(self.car_struct, angle=psi_car)
        rotated_struct_car += np.array([x_car,y_car]) # 4 points of car --> draw line between them

        # adding trailer's body to the environment
        rotated_struct_trailer = self.rotate_car(self.trailer_struct, angle=psi_trailer)
        rotated_struct_trailer += np.array([x_trailer,y_trailer]) # 4 points of trailer --> draw line between them

        temp_env = grd1.copy()

        # car and trailer body generation by assigning 8 to the corners of the vehicles 
        car_and_trailer =  np.concatenate( [rotated_struct_car, rotated_struct_trailer])

        for iix,iiy in car_and_trailer:
            
            temp_env[iix][iiy] = 8
                 
        return temp_env, car_and_trailer

    def trailer_kine(self, grd2, start_car, start_t, end):
      
        r=4 #wheel radius
        l=40 #wheel span 
        dt = 2 #time stamp

        r1 = 4 # Trailer wheel radius 
        dt1 = 2 # Trailer dt

        d1 = 50
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
        
        end_new_car = [] # tracks the end position of the path before parking 
        end_new_t = []

        prev = []                                                       # keeps the track of previous nodes
        grd_trc = [[-1 for col in range(800)] for row in range(800)]    # keeps the track of sequence of previously visited node
        dist_from_start = [[math.inf for col in range(800)] for row in range(800)] #inf matrix of 800*800

        current = start_car                         #represents the current node which is the starting point in our case 
        start_angle_c = 1.5708
        start_angle_t = 1.5708
        # start_angle = 0

        seen = []                               #tracks the visited points
        heap = []                               # here we are using heap instead of queue to optimize the time complexity of the program
        
        x_car = current[0]                          #x,y represent the starting coordinates
        y_car = current[1]

        x_t = start_t[0]
        y_t = start_t[1]

        dist_from_start[x_car][y_car] = 0               #distance for the starting point = 0
        # dist_from_start represents the distance from start point to the next point
        
        seen.append(current)                    #adding the visited point into the seen list
        
        found = False
        resign = False
        
        heapq.heappush(heap, [dist_from_start[x_car][y_car], [x_car, y_car], [x_t,y_t], start_angle_c, start_angle_t]) 
                
        while found == False and resign == False:       
            if len(heap) == 0:              #there is no path if heap is empty
                resign = True
                print("No path for you! sad!")
                break
            else:
                node = heapq.heappop(heap)      #pop and return the smallest value from the heap
                
                # a = node[0]
                xc = node[1][0]
                yc = node[1][1]
                xt = node[2][0]
                yt = node[2][1]

                # x_int, y_int = (int(x), int(y))

                start_angle_c = node[3]
                start_angle_t = node[4]

                # prev.append([x_int,y_int])
                prev.append([xc,yc])
                
                # end_new = [x_int,y_int] # tracks the end position of the path before parking
                end_new_car = [xc,yc]
                end_new_t = [xt, yt]

                if (grd2[xc][yc]==6):                  #Remeber, 6 is the value of the end position in our 2D grid
                    found = True
                    print("Path found")
                    break
                else:
                    for ii,item in enumerate(motion_conn): 
                        
                        theta_new_0 = start_angle_c + ((r/l)*(item[0] - item[1]) )*dt 
                        theta_new_1 = start_angle_t + ((r1/43)*(item[0] - item[1]) )*dt1

                        x1c = int(xc + (r/2)*(item[0] + item[1])*math.cos(theta_new_0)*dt)
                        y1c = int(yc + (r/2)*(item[0] + item[1])*math.sin(theta_new_0)*dt)

                        x1t = int(xt + (r1/2)*(item[0] + item[1])*math.cos(theta_new_1)*dt1)
                        y1t = int(yt + (r1/2)*(item[0] + item[1])*math.sin(theta_new_1)*dt1)

                        if 0 <= x1c and x1c< 799 and 0 <= y1c and y1c< 799 and grd2[x1c][y1c] != 1 and 0 <= x1t and x1t< 799 and 0 <= y1t and y1t< 799 and grd2[x1t][y1t] != 1:
                            cost_c = math.sqrt((x1c-xc)**2 +(y1c-yc)**2) #measures the distance between two neighbours

                            if(dist_from_start[xc][yc] + cost_c < dist_from_start[x1c][y1c]): #update the cost if new cost is greater than the previous cost
                                
                                dist_from_start[x1c][y1c] = dist_from_start[xc][yc] + cost_c
                                heapq.heappush(heap, [dist_from_start[x1c][y1c], [x1c, y1c],[x1t, y1t], theta_new_0, theta_new_1])
                                grd_trc[x1c][y1c] = [ii,[xc,yc], [xt,yt],theta_new_0, theta_new_1]
                   
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
                   
        ## Back Tracing of the generated path------------------------------------------------------
        if resign == False:

            counter = 0
            new_path_car = []
            new_path_t = []

            angle_list_car = []
            angle_list_t = []

            new_path_car.append(end_new_car)
            new_path_t.append(end_new_t)
            
            xc_ = end_new_car[0]    
            yc_ = end_new_car[1]
            xt_ = end_new_t[0]    
            yt_ = end_new_t[1]

            while ( int(xc_) != 150 ) or (int(yc_) != 150):
                if counter < 2000:
                    
                    gd = grd_trc[xc_][yc_] 
                    
                    x2c_ = int(gd[1][0])         # Back tracing the path.
                    y2c_ = int(gd[1][1])

                    x2t_ = int(gd[2][0])         # Back tracing the path.
                    y2t_ = int(gd[2][1])

                    new_path_car.append([x2c_, y2c_])
                    new_path_t.append([x2t_, y2t_])

                    angle_list_car.append(gd[3])
                    angle_list_t.append(gd[4])

                    counter+=1

                    xc_ = x2c_
                    yc_ = y2c_
                    xt_ = x2t_
                    yt_ = y2t_

                else:
                    break
                
            new_path_car.reverse()  # The path in the final_path will be from end to start to we will reverse it.
            new_path_t.reverse()

            angle_list_car.reverse()
            angle_list_t.reverse()
            
            return new_path_car, new_path_t, angle_list_car, angle_list_t
