"""
Author: Andrew Q. Pham, Victor Shia
Email: apham@g.hmc.edu, vshia@g.hmc.edu
Date of Creation: 2/26/20
Description:
    Particle filter implementation to filtering localization estimate
    This code is for teaching purposes for HMC ENGR205 System Simulation Lab 4
    Student code version with parts omitted.
"""


import matplotlib.pyplot as plt
import numpy as np
import shelve
from utils import *
from prediction import prediction_and_correction_step
from resample import resample_step


def moving_average(x, window = 10):
    return np.convolve(x, 1.0 * np.ones(window) / window, 'full')

def getYawVel(yawPrev, yawCurr):
    return wrap_to_pi((yawCurr - yawPrev))/DT

def main():
    """Run a PF on logged data from IMU and LiDAR moving in a box formation around a landmark"""

    filename="./shelve.out"
    my_shelf = shelve.open(filename, "n") # 'n' for new

    # filepath = "../"
    filename = "2020_2_26__16_59_7_filtered"
    data, is_filtered = load_data(filename)

    # Save filtered data so don't have to process unfiltered data everytime
    if not is_filtered:
        data = filter_data(data)
        save_data(data, filename+"_filtered.csv")

    # Load data into variables
    x_lidar = data["X"]
    y_lidar = data["Y"]
    z_lidar = data["Z"]
    time_stamps = data["Time Stamp"]
    lat_gps = data["Latitude"]
    lon_gps = data["Longitude"]
    yaw_lidar = data["Yaw"]
    pitch_lidar = data["Pitch"]
    roll_lidar = data["Roll"]
    x_ddot = data["AccelX"]
    y_ddot = data["AccelY"]

    x_ddot = moving_average(x_ddot)
    y_ddot = moving_average(y_ddot)

    for t in range(len(time_stamps)):
        yaw_lidar[t] = (-1*yaw_lidar[t] *  math.pi /180)

    lat_origin = lat_gps[0]
    lon_origin = lon_gps[0]

    #  Initialize filter
    """STUDENT CODE START"""
    particlesInit = np.zeros([PARTICLES, N])
    for i in range(PARTICLES):
        particlesInit[i,:] = [np.random.normal(0, .5), np.random.normal(0, .5), np.random.normal(0, .2), 
                                np.random.normal(0, .5),np.random.normal(0, math.pi/6), 1/PARTICLES]

    state_est_t_prev = [np.mean(particlesInit[:,0]), np.mean(particlesInit[:,1]), np.mean(particlesInit[:,2]), np.mean(particlesInit[:,3]), np.mean(particlesInit[:,4]), np.mean(particlesInit[:,5])]

    state_estimates = np.empty((N, len(time_stamps)))
    for i in range(N):
        state_estimates[i, 0] = state_est_t_prev[i]
    particlesAllTime =  np.zeros([PARTICLES, N, len(time_stamps)])
    gps_estimates = np.empty((2, len(time_stamps)))

    [gps_estimate_x, gps_estimate_y] = convert_gps_to_xy(lat_origin, lon_origin, lat_origin, lon_origin)

    gps_estimates[0,0] = gps_estimate_x
    gps_estimates[1,0] = gps_estimate_y
    """STUDENT CODE END"""

    #  Run filter over data
    for t, _ in enumerate(time_stamps):
        # Get control input
        """STUDENT CODE START"""
        u_t = np.empty([2, 1])
        u_t[0] = x_ddot[t]
        u_t[1] = getYawVel(yaw_lidar[t-1], yaw_lidar[t])
        """STUDENT CODE END"""
        z_t = np.empty([3, 1])
        z_t[0] = 5 - (y_lidar[t]* math.cos(wrap_to_pi(yaw_lidar[t])) + x_lidar[t]*math.sin(wrap_to_pi(yaw_lidar[t])))
        z_t[1] = -5 - (y_lidar[t]* math.sin(wrap_to_pi(yaw_lidar[t])) - x_lidar[t]*math.cos(wrap_to_pi(yaw_lidar[t])))
        z_t[2] = wrap_to_pi(yaw_lidar[t])
        # Prediction Step
        state_pred_t = prediction_and_correction_step(state_est_t_prev, u_t, z_t)
        resPart = resample_step(state_pred_t)

        #  For clarity sake/teaching purposes, we explicitly update t->(t-1)
        state_est_t_prev = np.array([np.average(resPart[:,0], weights = resPart[:,5]),
                                    np.average(resPart[:,1], weights = resPart[:,5]),
                                    np.average(resPart[:,2], weights = resPart[:,5]),
                                    np.average(resPart[:,3], weights = resPart[:,5]),
                                    np.average(resPart[:,4], weights = resPart[:,5]),
                                    1/N])

        
        # Log Data

        for i in range(N):
            state_estimates[i,t] = state_est_t_prev[i]

      
        x_gps, y_gps = convert_gps_to_xy(lat_gps=lat_gps[t],
                                         lon_gps=lon_gps[t],
                                         lat_origin=lat_origin,
                                         lon_origin=lon_origin)
        gps_estimates[:, t] = np.array([x_gps, y_gps])

    for key in dir():
        try:
            my_shelf[key] = eval(key)
        except Exception:
            #
            # __builtins__, my_shelf, and imported modules can not be shelved.
            #
            print('ERROR shelving: {0}'.format(key))
    my_shelf.close()


    """STUDENT CODE START"""
    # Plot here
    x_real = [0, 5, 10, 10, 10, 10, 10, 5, 0, 0, 0, 0]
    y_real = [0, 0, 0, 0, -5, -10, -10, -10, -10, -10, -5, 0]
    plt.plot(x_real, y_real)
    # plt.plot(gps_estimates[0],gps_estimates[1])
    plt.plot(state_estimates[0,:], state_estimates[1,:])
    plt.xlabel("X Coord (m)")
    plt.ylabel("Y Coord (m)")
    plt.title("Real vs Estimated Path")
    plt.show()
    """STUDENT CODE END"""
    return 0


if __name__ == "__main__":
    main()
