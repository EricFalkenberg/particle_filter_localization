from feature_handling import *
from progressbar import ProgressBar
import numpy as np
import copy
import math
import os

CAMERA, LANDMARK = 0, 1
PI               = math.pi
SCALE_FACTOR     = 0.2 
X_DIM_SIZE       = 200
Y_DIM_SIZE       = 200
Z_DIM_SIZE       = 200
HTHETA_DIM_SIZE  = 360
VTHETA_DIM_SIZE  = 360
RTHETA_DIM_SIZE  = 360

"""
Convert a given observation from x,y,z,htheta,vtheta,rtheta
to coordinates
"""
def convert_obs_to_coords(x_k, z_k):
    ## TODO: Make this attempt to relate an observation
    ##       to to other observations in an attempt to map it
    ##       to '3d' space
    x, y, z, htheta, vtheta, rtheta = z_k
    return np.array([x, y, z, htheta, vtheta, rtheta])

"""
Plot a point in our 6 dimensional grid
"""
def plot_point_in_grid(coords, m, val, point_type):
    mid_x, mid_y, mid_z = np.array([X_DIM_SIZE, Y_DIM_SIZE, Z_DIM_SIZE]) // 2
    x, y, z, _, _, _ = (coords//SCALE_FACTOR).astype(int)
    _, _, _, htheta, vtheta, rtheta = (np.degrees(coords)//SCALE_FACTOR).astype(int)
    if point_type == CAMERA:
        m[mid_x+x][mid_y+y][mid_z+z][CAMERA][htheta][vtheta][rtheta] = val
    elif point_type == LANDMARK:
        m[mid_x+x][mid_y+y][mid_z+z][LANDMARK] = val

"""
Initialize 6 dimensional state space
"""
def init_state_space():
    ## The line z_dim is paired with a single value 0.0 which represents
    ## the probability that any landmark exists at a point x,y,z2
    r_theta     = [0.25 for i in range(RTHETA_DIM_SIZE)]
    v_theta     = copy.deepcopy([r_theta for i in range(VTHETA_DIM_SIZE)])
    h_theta     = copy.deepcopy([[v_theta for i in range(HTHETA_DIM_SIZE)], 0.0])
    z_dim       = copy.deepcopy([h_theta for i in range(Z_DIM_SIZE)])
    y_dim       = copy.deepcopy([z_dim for i in range(Y_DIM_SIZE)])
    state_space = copy.deepcopy([y_dim for i in range(X_DIM_SIZE)])
    return state_space 

"""
P(z_k | x_k, m)
Computes the probability that we make a given observation
based on the current location and our map.
"""
def observation_model(z_k, x_k, m):
    #robot_coords = x_k[:-1]
    #robot_theta  = x_k[-1]
    #obs_coords   = convert_obs_to_coords(x_k, z_k)
    #low_bound    = robot_theta-PI/2
    #high_bound   = robot_theta+PI/2
    #target       = math.atan(np.divide(*(robot_coords-obs_coords)))

    ## If we are facing in the general direction of the point, then
    ## we can say that our observation is probably good.
    #if low_bound <= target <= high_bound:
    #    return 0.9
    #else:
    #    return 0.1
    return 0.9
"""
Compute the probability that the camera is at x_k based on 
all previous locations.
"""
def time_update(x_k, x_old):
    ## TODO: Calculate this recursively
    if np.linalg.norm(x_k[:3]-x_old[:3]) <= 1:
        return 0.9
    else:
        return 0.1

"""
Calculate the joint posterior for the robot state x_k
and map m at time k based on all observations Z_0:k
"""
def measurement_update(X, m, Z, x0):
    mid_x, mid_y, mid_z = np.array([X_DIM_SIZE, Y_DIM_SIZE, Z_DIM_SIZE]) // 2
    box_size = 1
    last_pos = X[-1]
    print mid_x, mid_y, mid_z                                                   ## TODO: DONT DIVIDE BY DECIMAL DUMB
    x, y, z = np.array([mid_x+last_pos[0], mid_y+last_pos[1], mid_z+last_pos[2]]) // SCALE_FACTOR
    x, y, z = int(x), int(y), int(z)
    print x, y, z
    progress = ProgressBar(max_value=27)
    count = 0
    print "Computing possible camera locations..."
    for dim_x in xrange(x-box_size, x+box_size+1):
        for dim_y in xrange(y-box_size, y+box_size+1):
            for dim_z in xrange(z-box_size, z+box_size+1):
                count += 1
                #progress.update(count)
                print dim_x, dim_y, dim_z
                for dim_ht in xrange(HTHETA_DIM_SIZE):
                    for dim_vt in xrange(VTHETA_DIM_SIZE):
                        for dim_rt in xrange(RTHETA_DIM_SIZE):
                            if dim_x < 0 or dim_x >= X_DIM_SIZE or \
                               dim_y < 0 or dim_y >= Y_DIM_SIZE or \
                               dim_z < 0 or dim_z >= Z_DIM_SIZE:
                                continue
                            if dim_x == 100 and dim_y == 100 and dim_z == 100:
                                print 'doing a thing'
                            prior = m[dim_x][dim_y][dim_z][CAMERA][dim_ht][dim_vt][dim_rt]
                            prior_odds = prior/(1-prior)
                            numerator = observation_model(Z[-1], X[-1], m) * prior_odds
                            denominator = 1
                            m[dim_x][dim_y][dim_z][CAMERA][dim_ht][dim_vt][dim_rt] = \
                                                                    numerator/denominator
    print m[100][100][100][CAMERA][0][0]
"""
The equation for updating all observations in our model
"""
def observation_update(X, m, frame_idx, frames):
    ## additive, zero mean, uncorrelated gaussian
    ## motion disturbance
    norm = np.linalg.norm(X[-1])
    x_hat = 1/norm*X[-1] if norm != 0 else 0
    v_k   = np.random.normal(loc=0.0, scale=0.01, size=3)    


##
## TODO: IMPLEMENT GRID UPDATE
##       (odds(old)+odds(update)) / numerator
##

def main():
    ## Map
    m = init_state_space() 
    ## Camera locations: x, y, z, htheta, vtheta, rtheta
    X = np.array([[0.0,0.0,0.0,0.0,0.0,0.0]])
    ## Plot point x_0 in m
    plot_point_in_grid(X[0], m, 0.99, CAMERA)
    ## Observation locations
    observations = np.array([])
    ## Process all frames 
    frames = process_images()  
    for frame_idx in range(len(frames)):
        ## Update observation model
        print "FRAME:", frame_idx
        observations = np.append(observations, frames[frame_idx][1])
        observation_update(X, m, frame_idx, frames)
        val = measurement_update(X, m, observations, X[0]) ## DO MAIN LOOP 
        print m[100][100][100][CAMERA][0][0]
main()
