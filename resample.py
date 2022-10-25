import numpy as np
from utils import *


def resampleHelp(weights):
    N = 200
    indexes = np.zeros(N)
    for i in range(N):
        r =  np.random.random()*np.sum(weights)
        j = 0
        wsum = weights[0]
        while(wsum < r):
            j += 1
            wsum += weights[j]
        indexes[i] = j
    indexes = indexes.astype(int)
    return indexes
    

def resample_step(particles):
    """Ressampling step for the PF

    Parameters:
    x_bar_t       (np.array)    -- the predicted state estimate of time t
    """

    """STUDENT CODE START"""
    indexes = resampleHelp(particles[:,5])
    particles[:] = particles[indexes]
    particles[:,5] /= np.sum(particles[:,5])
    """STUDENT CODE END"""

    return particles
    
  
