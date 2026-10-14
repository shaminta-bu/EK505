import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3

# TODO: replace with the angles/offsets given in the handout
THETA_01 = np.pi / 4  # rotation around z for T01
D_01 = 1.0            # translation along z for T01
THETA_12 = np.pi / 4  # rotation around x for T12
D_12 = 1.0            # translation along z for T12

# Rotation followed by translation (translation is along the rotated z axis)
T01 = SE3.Rz(THETA_01) * SE3.Tz(D_01)
T12 = SE3.Rx(THETA_12) * SE3.Tz(D_12)
T02 = T01 * T12


def main():
    print('T01 =\n', T01)
    print('T12 =\n', T12)
    print('T01*T12 =\n', T02)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    SE3().plot(ax=ax, frame='0', color='black', dims=[-2, 2])
    T01.plot(ax=ax, frame='1', color='red')
    T12.plot(ax=ax, frame='12', color='green')
    T02.plot(ax=ax, frame='2', color='blue')
    plt.show()


if __name__ == '__main__':
    main()
