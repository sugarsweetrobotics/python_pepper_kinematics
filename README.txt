# pepper_kinematics

## Description 

Originally, Pepper does not provide any inverse kinematics function. This provides simple inverse kinematics funciton.

## Example

    """
    Move work_pose first, then move 5 centimeters toward left (y axis positive side).
    """
    
    import time
    import numpy as np
    import naoqi as n
    import pepper_kinematics as pk
    
    host = 'nao.local'
    port = 9559
    
    m = n.ALProxy("ALMotion", host, port)
    m.setAngles(pk.left_arm_tags, pk.left_arm_work_pose, 1.0)
    
    time.sleep(1.0)
    
    current_angles = m.getAngles(pk.left_arm_tags)
    current_position, current_orientation = pk.left_arm_get_position(current_angles)
    
    target_position = current_position
    target_position[1] = target_position[1] + 0.05 # 5 cm toward left
    target_orientation = current_orientation # This is not supported yet
    
    target_angles = p.left_arm_set_position(current_angles, target_position, target_orientation)
    if target_angles:
       m.setAngles(pk.left_arm_tags, target_angles, 1.0)


## How to install
    sudo pip install pepper_kinematics

## Copyright
* author: Yuki Suga
* copyright: Yuki Suga @ ssr.tokyo
* license: GPLv3

