import numpy as np
import matplotlib.pyplot as plt
import cv2
from trailer_env import Environment

car_start = [150,150] # Starting position of the Car
trailer_start = [100,100] # Starting position of the Trailer
car_end = [629, 289] #end point for car
res = Environment() #returns the environment 
env = res.obs_gen() # generated obstacles 

##-------------------------------------------------------------------------------------------------------

path_car, path_trailer, angle_list_car, angle_list_trailer = res.trailer_kine(env, car_start, trailer_start, car_end) 

##-------------------------------------------------------------------------------------------------------
## Normal Path printing-------------------------------------------------------------------------------------

for jj in range(len(path_car)-1):
    psi_car = angle_list_car[jj]
    psi_trailer = angle_list_trailer[jj]

    updated_env, rotated_struct_all = res.render(env, path_car[jj][0],path_car[jj][1],path_trailer[jj][0],path_trailer[jj][1],psi_car, psi_trailer)
    imS = cv2.resize(updated_env, (600, 600))  
    cv2.imshow('environment', imS)
    key = cv2.waitKey(30)

updated_env, rotated_struct_all = res.render(env, path_car[jj][0],path_car[jj][1],path_trailer[jj][0],path_trailer[jj][1],psi_car, psi_trailer)
imS = cv2.resize(updated_env, (600, 600))  
cv2.imshow('environment', imS)
key = cv2.waitKey(5000)

##Currently Working on this Problem----------------------------------------------------------------------------------
##--------------------------- Trailer Parallel Parking Printing-----------------------------------------------------------------------------
# psi_resolution = 0.0174533 # 0.9 degree change divided into 50 in positive and 50 in negative direction changes 
# psi = 1.5708 

# # print(len(parking_path))

# for jj in range(int(len(parking_path)/2)):
#     psi += psi_resolution

#     updated_env, rotated_struct = res.render(env, parking_path[jj][0],parking_path[jj][1],psi)
#     # print("I am good!")
#     imS = cv2.resize(updated_env, (600, 600))  
#     cv2.imshow('environment', imS)
#     key = cv2.waitKey(30)

# for kk in range(1, int(len(parking_path)/2)):
#     psi -= psi_resolution

#     updated_env, rotated_struct = res.render(env, parking_path[jj+kk][0],parking_path[jj+kk][1],psi)
#     # print("I am good!")
#     imS = cv2.resize(updated_env, (600, 600))  
#     cv2.imshow('environment', imS)
#     key = cv2.waitKey(30)

# updated_env, rotated_struct = res.render(env, path[jj+1][0],path[jj+1][1],hard_angles)
# # print("I am good!")
# imS = cv2.resize(updated_env, (600, 600))  
# cv2.imshow('environment', imS)
# key = cv2.waitKey(3000)

##Printing x, y cordinates----------------------------------------------------
# a = []
# b = []

# final_path =path_trailer 

# for iii in range(len(final_path)):
#     a.append(final_path[iii][0])
#     b.append(final_path[iii][1])


# plt.scatter(a, b,c='red',alpha=0.1)
# plt.title('Position Plot --> Trailer Path (Trailer Problem)')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.show()

