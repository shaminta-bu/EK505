'''
Robot models (SCARA, cart-pole) and their hand-derived forward-kinematics functions.
'''

from os import path

from roboticstoolbox.models.URDF.URDFRobot import URDFRobot
from spatialmath import SE3

_here = path.dirname(path.abspath(__file__))
scara = URDFRobot(path.join(_here, 'scara.urdf'))
cartpole = URDFRobot(path.join(_here, 'cartpole.urdf'))


def scara_fkine(q_input):
    '''
    Computes the forward kinematics of the SCARA robot by explicitly concatenating transformations
    '''

    pose = (SE3(0, 0, 0.3) * SE3.Rz(q_input[0])
            * SE3(0.90, 0, 0.08) * SE3.Rz(q_input[1])
            * SE3(1.02, 0, 0.08) * SE3.Tz(q_input[2])
            * SE3(0, 0, -0.45))
    return pose


def cartpole_fkine(q_input):
    '''
    Computes the forward kinematics of the cart-pole robot by explicitly
    concatenating transformations
    '''
    pose = (SE3(0, 0, 0.15) * SE3.Tx(q_input[0])
            * SE3(0, 0.28, 0.1) * SE3.Ry(q_input[1])
            * SE3(0, 0, 1.0))
    return pose
