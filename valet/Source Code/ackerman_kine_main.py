import numpy as np
import matplotlib.pyplot as plt
import random
import collections
import math
import heapq
import cv2

from ackerman_environment import Environment

start = [100,100] # car's starting point
end = [629, 289] #end point for ackerman steering

res = Environment()
env = res.obs_gen()

##-------------------------------------------------------------------------------------------------------

path, angle_list = res.ackerman_kine(env, start, end) 

##Angle calculation from path-------------------------------------------------
# for numi in range(len(path)-1):

#     angle_list.append(math.atan(  (path[numi+1][1] - path[numi][1])/(path[numi+1][0] - path[numi][0])   ))

# angle_list.append(1.5708)
##------------------------------------------------------------------------------------------------------------
## Parallel Parking path generation for Ackerman Steering 
#  
parking_path = []
hard_angle = np.deg2rad(90) #take negative lists of angles  
xop, yop = [629,289]
xip, yip = [729,189]
xmid, ymid = [691, 241]

range_x = xop-xip
range_y = yop-yip

slop1 = abs((ymid-yop)/(xmid-xop))
slop2 = abs((yip-ymid)/(xip-xmid))

counter = 1
# parking_path.append([xop,yop])

x_,y_ = xop,yop
while True:
    if (x_ == xmid and y_ == ymid):
        break
    if(counter<1000):
        x_ = xop + counter
        y_ = yop - slop1*counter
        parking_path.append([x_,y_])
        counter+=1

counter = 1
while True:
    if (x_ == xip and y_ == yip):
        break
    if(counter<1000):
        x_ = xmid + counter
        y_ = ymid - slop2*counter
        parking_path.append([x_,y_])
        counter+=1

## Normal Path printing-------------------------------------------------------------------------------------

for jj in range(len(path)-1):
    psi = angle_list[jj]

    updated_env, rotated_struct = res.render(env, path[jj][0],path[jj][1],psi)
    imS = cv2.resize(updated_env, (600, 600))  
    cv2.imshow('environment', imS)
    key = cv2.waitKey(30)

updated_env, rotated_struct = res.render(env, path[jj+1][0],path[jj+1][1],1.5708)
imS = cv2.resize(updated_env, (600, 600))  
cv2.imshow('environment', imS)
key = cv2.waitKey(5000)

##---------------------------Parallel Parking Printing-----------------------------------------------------------------------------
psi_resolution = 0.0174533 # 0.9 degree change divided into 50 in positive and 50 in negative direction changes 
psi = 1.5708 

# print(len(parking_path))

for jj in range(int(len(parking_path)/2)):
    psi += psi_resolution

    updated_env, rotated_struct = res.render(env, parking_path[jj][0],parking_path[jj][1],psi)
    # print("I am good!")
    imS = cv2.resize(updated_env, (600, 600))  
    cv2.imshow('environment', imS)
    key = cv2.waitKey(30)

for kk in range(1, int(len(parking_path)/2)):
    psi -= psi_resolution

    updated_env, rotated_struct = res.render(env, parking_path[jj+kk][0],parking_path[jj+kk][1],psi)
    # print("I am good!")
    imS = cv2.resize(updated_env, (600, 600))  
    cv2.imshow('environment', imS)
    key = cv2.waitKey(30)

##Printing x, y cordinates----------------------------------------------------
a = []
b = []

final_path = path + parking_path

for iii in range(len(final_path)):
    a.append(final_path[iii][0])
    b.append(final_path[iii][1])


plt.scatter(a, b,c='red',alpha=0.1)
plt.title('Position Plot --> Ackermann Drive')
plt.xlabel('x')
plt.ylabel('y')
plt.show()