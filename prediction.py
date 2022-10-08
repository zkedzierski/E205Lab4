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
    x_bar_t = np.array([x_t_prev[0] + x_t_prev[2]*DT],
                       [x_t_prev[1] + x_t_prev[3]*DT],
                       [x_t_prev[2] + u_t_noiseless[0]*math.cos(wrap_to_pi(x_t_prev[4]))*DT],
                       [x_t_prev[3] + u_t_noiseless[0]*math.sin(wrap_to_pi(x_t_prev[4]))*DT],
                       [((x_t_prev[4]) + (u_t_noiseless[1]*DT))])
    """STUDENT CODE END"""
    return x_bar_t


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
    # Prediction step
    x_bar_t = x_t_prev + np.random.rand(5).tranpose()

    # Weighting
    """STUDENT CODE END"""

    return x_bar_t
