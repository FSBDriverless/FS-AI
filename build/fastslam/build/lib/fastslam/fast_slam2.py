"""

FastSLAM 2.0 example

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
from algorithm import *
from draw import draw_animation

#  Simulation parameter
Q_sim = np.diag([0.3, np.deg2rad(2.0)]) ** 2
R_sim = np.diag([0.5, np.deg2rad(10.0)]) ** 2
OFFSET_YAW_RATE_NOISE = 0.01

SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling

show_animation = True

def calc_input(time):
    if time <= 3.0:  # wait at first
        v = 0.0
        yaw_rate = 0.0
    else:
        v = 1.0  # [m/s]
        yaw_rate = 0.1  # [rad/s]

    u = np.array([v, yaw_rate]).reshape(2, 1)

    return u


def observation(xTrue, xd, u, RFID):
    # calc true state
    xTrue = motion_model(xTrue, u)

    # add noise to range observation
    z = np.zeros((3, 0))

    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])

        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise
            angle_noise = np.random.randn() * Q_sim[1, 1] ** 0.5
            angle_with_noise = angle + angle_noise  # add noise
            zi = np.array([dn, pi_2_pi(angle_with_noise), i]).reshape(3, 1)
            z = np.hstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
    ud2 = u[1, 0] + np.random.randn() * R_sim[
        1, 1] ** 0.5 + OFFSET_YAW_RATE_NOISE
    ud = np.array([ud1, ud2]).reshape(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud

def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [15.0, 15.0],
                     [10.0, 20.0],
                     [3.0, 15.0],
                     [-5.0, 20.0],
                     [-5.0, 5.0],
                     [-10.0, 15.0]
                     ])
    n_landmark = RFID.shape[0]
    print("Landmark: %s" % n_landmark);

    # State Vector [x y yaw v]'
    xTrue = np.zeros((STATE_SIZE, 1))  # True state

    # history
    hxTrue = xTrue

    slam = Slam() 

    while SIM_TIME >= time:
        time += 0.1
        u = calc_input(time)

        xTrue, z, xDR, ud = observation(xTrue, slam.xDR, u, RFID)

        slam.iteration(0.1, ud, z)

        # store data history
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:  # pragma: no cover
            draw_animation(RFID, N_PARTICLE, hxTrue, slam.hxDR, slam.hxEst, z, slam.xEst, slam.particles)


if __name__ == '__main__':
    main()
