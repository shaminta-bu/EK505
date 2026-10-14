'''
Utility functions for interpolating and animating robot joint-space trajectories.
'''

import numpy as np


def interp_pingpong(q_waypoints, nb_per_segment=100):
    '''
    Given a sequence of waypoints (as a list of 1-D vectors), returns a sequence of
    configurations obtained by
    1) chaining segments that linearly interpolate between consecutive waypoints
       (using nb_per_segment points per segment),
    2) adding the reversed trajectory to obtain the "ping-pong" effect.
    '''

    q_traj = []
    for i in range(len(q_waypoints) - 1):
        q1 = np.array(q_waypoints[i])
        q2 = np.array(q_waypoints[i + 1])
        start_j = 1 if i > 0 else 0
        for j in range(start_j, nb_per_segment):
            t = j / (nb_per_segment - 1) if nb_per_segment > 1 else 0
            q = (1 - t) * q1 + t * q2
            q_traj.append(q)

    # Add the reversed trajectory for the ping-pong effect
    for i in range(len(q_traj) - 2, 0, -1):
        q_traj.append(q_traj[i])

    return np.array(q_traj)
