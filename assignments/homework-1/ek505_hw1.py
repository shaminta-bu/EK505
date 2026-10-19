'''
Forward-kinematics tests and Swift animation demo for the EK505 homework robots.
'''

import os
import sys
import numpy as np
import swift

# Robot models are shared across assignments in ../staples
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'staples'))
import ek505_models as models
import ek505_utilities as utilities


def fkine_test(robot, robot_fkine):
    '''
    Helper function for the forward kinematic test functions specific to each robot.
    '''
    q_test = np.random.rand(5, 3)

    for q in q_test:
        print('## q =', q)
        print('### Robotics Toolbox pose')
        print(robot.fkine(q))
        print('### Direct pose')
        print(robot_fkine(q))


def cartpole_fkine_test():
    '''
    Show the results of the forward kinematics computed with both the
    Robotics Toolbox and the hand-written pose composition for the Cart-Pole robot
    '''
    print('# Cart-Pole robot')
    fkine_test(models.cartpole, models.cartpole_fkine)


def scara_fkine_test():
    '''
    Show the results of the forward kinematics computed with both the
    Robotics Toolbox and the hand-written pose composition for the SCARA robot
    '''
    print('# SCARA robot')
    fkine_test(models.scara, models.scara_fkine)


def animation_interpolation_test(nb_cycles=3):
    '''
    Define a mock pick-and-place sequence by using a sequence of waypoints
    interpolated with interp_pingpong()
    '''
    robot = models.scara

    # Joint order: [base_to_link1, link1_to_link2, link2_to_plunger]
    q_home = [0.0, 0.0, 0.0]
    q_pick_up = [0.0, 0.0, -0.25]
    q_swing_over = [-1.2, -1.5, -0.25]
    q_place_down = [-1.2, -1.5, 0.0]

    q_waypoints = [
        q_home,
        q_pick_up,
        q_swing_over,
        q_place_down
        ]

    q_traj = utilities.interp_pingpong(q_waypoints, nb_per_segment=50)

    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot)

    for _ in range(nb_cycles):
        for q in q_traj:
            robot.q = q
            env.step(0.02)

    env.hold()


if __name__ == '__main__':
    #animation_interpolation_test()
    scara_fkine_test()
    cartpole_fkine_test()
