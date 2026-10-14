from os import path

import numpy as np
import sympy as sp
import sympy.physics.mechanics as me
from roboticstoolbox.models.URDF.URDFRobot import URDFRobot # robot imported from the URDF using the RTB
from spatialmath import SE3 # for computing the end effector pose
from spatialmath.base import skewa

import ek505_activity_utilities as ut

me.init_vprinting(use_unicode=True)

robot = URDFRobot(
    path.join(path.abspath(path.curdir), 'spherical_centered.urdf'))


def robot_fkine_ee(q):
    '''
    Forward kinematics for the spherical (centered) robot
    '''

    return SE3.Rz(q[0]) * SE3.Rx(q[1]) * SE3.Tz(0.8) * [0, 0, 0]
    # computer end effector pose (multiply by the coordinates of the origin at the end)
    # each factor is shown as a matrix, but for python is an object of class SE3
    # transformation * a 3-vector gives a 3-vector.

def robot_fkine_ee_jacob(q, qd):
    '''
    Compute the Jacobian applied to joint velocities by using derivatives of transformations
    '''

    # End effector position in the end effector frame
    p_ee = np.array([[0], [0], [0], [1]])
    # p_b0_ee = SE3.Rz([0]) * SE3.Rx(q[1]) * SE3.Tz(0.8) * [0, 0, 0]
    # p_b0_ee = SE3.Rz([0]).A @ SE3.Rx([q1]).A @ SE3.Tz(0.8).A @ p_ee
    # d/dt Rz(q0) = Rz(q0) @ hat(e_z) * qd0 ;  d/dt Rx(q1) = Rx(q1) @ hat(e_x) * qd1
    # skewa takes a twist [vx, vy, vz, wx, wy, wz] and returns the 4x4 se(3) matrix
    p_b0_ee_dot = SE3.Rz(q[0]).A @ skewa([0, 0, 0, 0, 0, qd[0]]) @ SE3.Rx(q[1]).A @ SE3.Tz(0.8).A @ p_ee \
                + SE3.Rz(q[0]).A @ SE3.Rx(q[1]).A @ skewa([0, 0, 0, qd[1], 0, 0]) @ SE3.Tz(0.8).A @ p_ee
    # keep only the 3D velocity (drop the homogeneous 4th row, which is 0)
    p_b0_ee_dot = p_b0_ee_dot[:3, :]
    # TODO: replace with the transformation computed by using matrix multiplication
    # and skew-symmetric matrices
    return p_b0_ee_dot


def main():
    # TODO: create q as a symbolic array
    q = sp.symbols('q0 q1')


    print('# Jacobian from fkine')
    # TODO: Replace with jacobian of the Forward Kinematics for the end effector position
    f = sp.Matrix(robot_fkine_ee(q))   # NumPy (object) array -> SymPy Matrix
    jacob_fkine = sp.simplify(f.jacobian(q))
    ut.print_matrix(jacob_fkine)

    print('# Jacobian from transformations')
    # TODO: Compute Jacobian one column at a time using robot_fkine_ee_jacob
    jacob_transf_col0 = robot_fkine_ee_jacob(q, [1, 0])
    jacob_transf_col1 = robot_fkine_ee_jacob(q, [0, 1])
    jacob_transf = sp.simplify(sp.Matrix(np.hstack((jacob_transf_col0, jacob_transf_col1))))
    ut.print_matrix(jacob_transf)

    print('# Build-in Jacobian')

    # TODO: Use SymPy to simplify the expression
    jacob_rtb = sp.simplify(sp.Matrix(robot.jacob0(q)))
    ut.print_matrix(jacob_rtb)


if __name__ == '__main__':
    main()
