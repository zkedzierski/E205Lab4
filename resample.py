import numpy as np
from utils import *

def resample_step(particles):
    """Ressampling step for the PF

    Parameters:
    x_bar_t       (np.array)    -- the predicted state estimate of time t
    """

    """STUDENT CODE START"""
    # particlesRes = np.zeros([PARTICLES, N])
    # for i in range(PARTICLES):
    #     r = np.random.rand()*sumW
    #     j = 1
    #     wSum = particles[0:5] 
    #     while (wSum < r):
    #         j += 1
    #         wSum += particles[j:5] 
    weights = particles[:,5]
    resWeights = np.random.Generator.choice(weights, size = PARTICLES, replace = False, p = None)
    for i in range(PARTICLES):
        particles[i,5] = resWeights[i]
    """STUDENT CODE END"""

    return particles
