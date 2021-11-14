import numpy as np
import matplotlib.pyplot as plt
import cv2

from diwheel_environment import Environment

start = [100,100] # starting point of the car
end = [729, 194] #end point of the car

res = Environment() # Environment Generation 
env = res.obs_gen() # Obstacle Generation

##-------------------------------------------------------------------------------------------------------
#planner is also implemented in the environment
# diwheel_kine --> Kinematics Planning for Diwheek Robot

path, angle_list = res.diwheel_kine(env, start, end) 

## printing-------------------------------------------------------------------------------------

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

##--------------------------------------------------------------------------------------------------
# Plot----------------------------------

a = []
b = []

for iii in range(len(path)):
    a.append(path[iii][0])
    b.append(path[iii][1])

plt.scatter(a, b,c='red',alpha=0.1)
plt.title('Position Plot --> Differencial Drive')
plt.xlabel('x')
plt.ylabel('y')
plt.show()

