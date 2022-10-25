import numpy as np
from utils import *


def propogate_state(x_t_prev, u_t_noiseless):
    """Propagate/predict the state based on chosen motion model

    Parameters:
    x_t_prev (np.array)  -- the previous state estimate
    u_t (np.array)       -- the current control input

    Returns:
    x_bar_t (np.array)   -- the predicted state
    """
    """STUDENT CODE START"""
    x_bar_t = np.zeros([6,1])
    x_bar_t[0] = x_t_prev[0] + x_t_prev[2]*DT + np.random.normal(0, .1)
    x_bar_t[1] = x_t_prev[1] + x_t_prev[3]*DT + np.random.normal(0, .1)  
    x_bar_t[2] = x_t_prev[2] + (u_t_noiseless[0] + np.random.normal(0, .1))*math.cos(wrap_to_pi(x_t_prev[4]))*DT + np.random.normal(0, .1) 
    x_bar_t[3] = x_t_prev[3] + (u_t_noiseless[0] + np.random.normal(0, .1))*math.sin(wrap_to_pi(x_t_prev[4]))*DT + np.random.normal(0, .1) 
    x_bar_t[4] = x_t_prev[4] + (u_t_noiseless[1] + np.random.normal(0, .1))*DT + np.random.normal(0, .1) 
    x_bar_t[5] = x_t_prev[5]
    """STUDENT CODE END"""
    return x_bar_t


def calcPZX(x_t, z_t): 
    cov = math.sqrt(2*math.pi)*np.identity(3)
    det = np.linalg.det((cov))
    x_t = np.array([x_t[0], x_t[1], x_t[4]])
    pZX = (1/det) * math.exp(-.5 * (z_t - x_t).transpose() @ np.linalg.inv(np.identity(3)) @ (z_t - x_t))
    return pZX

def prediction_and_correction_step(x_t_prev, u_t, z_t):
    """Compute the prediction and correction (re-weighting) for the PF

    Parameters:
    x_t_prev (np.array)         -- the previous state estimate
    u_t (np.array)              -- the control input
    z_t (np.array)              -- the current measurement

    Returns:
    x_bar_t (np.array)          -- the predicted state estimate of time t
    """

    """STUDENT CODE START"""
    # Prediction & Correction steps
    particles = np.zeros([PARTICLES, N])
    sumW = 0
    for i in range(PARTICLES):
        x_bar_t = propogate_state(x_t_prev, u_t)
        particles[i,:] = x_bar_t.transpose()
        particles[i, N-1] = calcPZX(x_bar_t, z_t)
        sumW += calcPZX(x_bar_t, z_t)
    particles[:,5] = particles[:,5]/sumW

    """STUDENT CODE END"""

    return particles
