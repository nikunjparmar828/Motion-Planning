import numpy as np
import matplotlib.pyplot as plt
import random
import collections
import cv2
import time
from IPython.display import clear_output
import heapq

from  wild_env_prm import Environment1
from  wild_env_prm import prm_planner
from  wild_env_astar import Environment
from  wild_env_astar import astar_planner
import prm_test
# from prm_test import Node
from prm_test import prm_planning

density = 5 # Obstacles density in % in percentage

bush_env_obj = Environment()


bush_env, bush_list_with_state = bush_env_obj.bush_gen(density)


start = [10,10]

##------------------------------------------------------------------------------------------------------------
## threading can be a sraight forward way to solve the problem but I am not doing it here
##------------------------------------------------------------------------------------------------------------
time_60 = 0
time_20 = 0
total_time = 0

heap = []

bushes_man = []
updated_env = bush_env

def rand_fire(bush_list_with_state, updated_env, heap):
    global time_60
    global bushes_man
    if(time_60>=60 or time_60==0):
        bush_list_with_state, updated_env, bush_at_fire = bush_env_obj.rand_fire_at_60_sec(updated_env, bush_list_with_state)
        heapq.heappush(heap, bush_at_fire)
        bushes_man.append(bush_at_fire)
        time_60 = 0
    return bush_list_with_state, updated_env, heap


def neighbours_on_fire(bush_list_with_state, updated_env, heap):
    global time_20
    global bushes_man
    if(time_20>=20):
        bush_list_with_state, updated_env, child_bushes_at_fire = bush_env_obj.neighbour_fire_at_20_sec(updated_env, bush_list_with_state)
        
        for bushes_num in range(len(child_bushes_at_fire)):
            heapq.heappush(heap, child_bushes_at_fire[bushes_num])
            bushes_man.append(child_bushes_at_fire[bushes_num])
        
        time_20 = 0
    return bush_list_with_state, updated_env, heap

def end_state_update(bush_at_fire, updated_env, bush_list_with_state):
    bush_list_with_state, updated_env = bush_env_obj.end_state_update(bush_at_fire, updated_env, bush_list_with_state)

    return bush_list_with_state, updated_env

if(time_60>=60 or time_60==0):
    bush_list_with_state, updated_env, heap = rand_fire(bush_list_with_state, updated_env, heap)
    time_60 = 0 # just to be safe

bush_counter = 0
temp_path = []

while total_time<3600: 
    total_time+=2 # waitkey --> 200 means 1 sec so 1000 waitkey means 5 seconds 
    time_60+=2
    time_20+=2

    if (time_20>=20):
        bush_list_with_state, updated_env, heap = neighbours_on_fire(bush_list_with_state, updated_env, heap)
        time_20 = 0 # being safe 

    if(time_60>=60):
        bush_list_with_state, updated_env, heap = rand_fire(bush_list_with_state, updated_env, heap)
        time_60 = 0 # just to be safe

    try:
        got_data = bushes_man[bush_counter]
    except:
        got_data = 'null'

    if (got_data == 'null'): #len(heap)==0 or
        print("No fire found!")
        continue
    else:
        # bush_at_fire = heapq.heappop(heap) 
        path_obj = prm_planner()
        bush_at_fire = bushes_man[bush_counter]
        bush_counter+=1

        end = bush_at_fire[0]

        
        path_obj = astar_planner()

        inflated_env = path_obj.inflated_env(bush_env.copy(), bush_list_with_state)
##------------------------------------------------------PRM Approach--------------------------------------------------------------------
        ## counting obstacles cells in the inflated environment 

        obs_list_inflated_x = []
        obs_list_inflated_y = []

        for i_infl in range(250):
            for j_infl in range(250):
                if inflated_env[i_infl][j_infl] == 1:
                    obs_list_inflated_x.append(i_infl)
                    obs_list_inflated_y.append(j_infl)
        
        ##---------------------------------------------------------          

        # temp_path, angle_list = path_obj.path_generation(inflated_env, start, end)
        robot_size = 5
        rx, ry = prm_test.prm_planning(start[0], start[1], end[0], end[1], obs_list_inflated_x, obs_list_inflated_y, robot_size)

        for iii in rx:
            for jjj in ry:
                temp_path.append([iii,jjj])

        temp_path, angle_list = path_obj.path_generation(inflated_env, start, end)
        #### Visualization--------------------------------------------------------------------------------------------- 
        
        for jj in range(len(temp_path)-1):
            
            psi = angle_list[jj]
            updated_env_print, rotated_struct = bush_env_obj.render(updated_env.copy(), temp_path[jj][0],temp_path[jj][1],psi)
            # plt.imshow(updated_env)
        
            imS = cv2.resize(updated_env_print, (600,600))    
            cv2.imshow('environment',imS)
            cv2.waitKey(200)
            time.sleep(0.5)
            total_time+=1 
            time_60+=1
            time_20+=1 

            if (time_20>=20):
                bush_list_with_state, updated_env, heap = neighbours_on_fire(bush_list_with_state, updated_env, heap)
                time_20 = 0 # being safe 

            if(time_60>=60):
                bush_list_with_state, updated_env, heap = rand_fire(bush_list_with_state, updated_env, heap)
                time_60 = 0 # just to be safe      
        cv2.waitKey(1000)
        time.sleep(0.1)
        start = temp_path[jj]
        jj = 0
        #extingushing the fire
        bush_list_with_state, updated_env = end_state_update(bush_at_fire, updated_env, bush_list_with_state)
