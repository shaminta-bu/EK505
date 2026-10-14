from os import path

import numpy as np
import swift
from roboticstoolbox.models.URDF.URDFRobot import URDFRobot

robot = URDFRobot(path.join(path.abspath(path.curdir), 'spherical.urdf'))

DT = 0.05
TIME_VARYING = False  # set True for optional question 2.1


def main():
    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot)

    t = 0.0
    while True:
        if TIME_VARYING:
            robot.q = [np.sin(t), 0.5 * np.sin(2 * t)]
        else:
            robot.q = [0, 0]  # change these to see the effect of each joint
        env.step(DT)
        t += DT


if __name__ == '__main__':
    main()
