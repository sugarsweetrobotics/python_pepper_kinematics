import numpy as np
import scipy as sp
from scipy import linalg

import forward_kinematics as fk
import inverse_kinematics as ik

right_arm_tags = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
right_arm_initial_pose = [1.0, -0.2, 1.57-0.2, 1.0, -1.57]
right_arm_work_pose = [0.8, -0.2, 1.57-0.2, 0.9, -1.57]

_inverse_case = [1.0, -1.0, -1.0, -1.0, -1.0]

left_arm_tags = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
left_arm_initial_pose = [p[0] * p[1] for p in zip(right_arm_initial_pose, _inverse_case)]
left_arm_work_pose = [p[0] * p[1] for p in zip(right_arm_work_pose, _inverse_case)]

    
def right_arm_get_position(angles):
    """
    Just calculate the position when joints on the pepper's right arm is in given positions

    Args:
      angles : Angles of right arm joints (list of 5 double values. unit is radian)
    
    Returns:
      A tuple of two arrays (position, orientation). orientation is presented as Matrix. Unit = meter.
      
      (position, orientation) = (np.array([position_x, position_y, position_z]), np.array([[R00, R01, R02], [R10, R11, R12], [R20, R21, R22]]))
    """
    return fk.calc_fk_and_jacob(angles, jacob=False, right=True)

def left_arm_get_position(angles):
    """
    Just calculate the position when joints on the pepper's left arm is in given positions

    Args:
      angles : Angles of left arm joints (list of 5 double values. unit is radian)
    
    Returns:
      A tuple of two arrays (position, orientation). orientation is presented as Matrix. Unit = meter.
      
      (position, orientation) = (np.array([position_x, position_y, position_z]), np.array([[R00, R01, R02], [R10, R11, R12], [R20, R21, R22]]))
    """
    return fk.calc_fk_and_jacob(angles, jacob=False, right=False)

def right_arm_set_position(angles, target_pos, target_ori, epsilon=0.0001):
    """
    Just calculate the joint angles when the Pepper's right hand position is in the given position
    
    Args:
      angles : Use the initial position of calculation. Unit = radian
      target_pos : List. [Px, Py, Pz]. Unit is meter.
      target_ori : np.array([[R00,R01,R02],[R10,R11,R12],[R20,R21,R22]])
      epsilon    : The threshold. If the distance between calculation result and target_position is lower than epsilon, this returns value.
    
    Returns:
      A list of joint angles (Unit is radian). If calculation fails, return None.
    """
    return ik.calc_inv_pos(angles, target_pos, target_ori, epsilon, right=True)

def left_arm_set_position(angles, target_pos, target_ori, epsilon = 0.0001):
    return ik.calc_inv_pos(angles, target_pos, target_ori, epsilon, right=False)


