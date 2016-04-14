import numpy as np
import math

PI = math.pi

def convert_obs_to_coords(x_k, z_k, Z):
    pass

def observation_model(x_k, m):
    pass

def time_update():
    pass ## Recursive

def measurement_update(X, m, Z, x0):
    numerator   = observation_model(X[-1], m) * time_update()
    denominator = 1 ## Probability that our observation occured given
                    ## observation history (assuming 1 for now)
    return numerator/denominator

def observation_update(X, m, feature, matched):
    ## additive, zero mean, uncorrelated gaussian
    ## motion disturbance
    x_hat = 1/np.linalg.norm(X[-1])*X[-1]
    v_k   = np.random.normal(loc=0.0, scale=0.01, size=3)
    
 
def main():
    ## Observations
    z = np.array([])
    ## Landmark locations
    m = np.array([])
    ## Camera locations: x, y, z, htheta, vtheta, rtheta
    X = np.array(np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]))
    ## Observation dimensions: r, theta
    ## Test observations at (1,1) and (1,-1)
    observations = np.array([[math.sqrt(2), -pi/8], [math.sqrt(2),pi/8]])
    ## Current timestep
    timestep = 0
    ## Update observation model
    observation_update(X, observations, m, [])
    
main()
