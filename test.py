import os, time, math
import numpy as np
import pepper_kinematics as k
import naoqi

host = "localhost"
port = 51669


def calc_jacob(angles):
    epsilon = 0.01
    pos, ori = k.left_arm_get_position(angles)
    for i, angle in enumerate(angles):
        samples = [a for a in angles]
        samples[i] = samples[i] + epsilon
        pos_, ori_ = k.left_arm_get_position(samples)
        error = pos_ - pos
        print error.transpose() / epsilon

def left_arm_ik_continuous(verbose=False):
    work_pose = k.left_arm_work_pose
    pos, ori = k.left_arm_get_position(work_pose)
    poss = []
    d = 32
    r = 0.05
    time.sleep(5)
    poss = []
    pos[1] = pos[1] + r
    poss.append(np.copy(pos))
    p = np.copy(pos)
    p[1] = p[1] + r
    poss.append(p)
    poss.append(np.copy(pos))
    p = np.copy(pos)
    p[1] = p[1] - r
    poss.append(p)
    poss.append(np.copy(pos))
    p = np.copy(pos)
    p[2] = p[2] - r
    poss.append(p)
    poss.append(np.copy(pos))
    p = np.copy(pos)
    p[2] = p[2] + r
    poss.append(p)
    poss.append(np.copy(pos))

    motion = naoqi.ALProxy("ALMotion", host, port)
    print k.left_arm_work_pose
    motion.setAngles(k.left_arm_tags, work_pose, 1.0)
    time.sleep(1.0)
    for i in range(2):
        for p in poss:
            angles = motion.getAngles(k.left_arm_tags, True)
            target_angles = k.left_arm_set_position(angles, p, ori)
            a = [a for a in target_angles]
            motion.setAngles(k.left_arm_tags, a, 1.0)
            time.sleep(0.5)

    poss = []
    for i in range(d):
        p = np.copy(pos)
        th = math.pi * 2 / d * i
        p[1] = p[1] + r * math.cos(th)
        p[2] = p[2] + r * math.sin(th)
        poss.append(p)


    motion = naoqi.ALProxy("ALMotion", host, port)
    print k.left_arm_work_pose
    motion.setAngles(k.left_arm_tags, work_pose, 1.0)
    time.sleep(1.0)
    for i in range(5):
        for p in poss:
            angles = motion.getAngles(k.left_arm_tags, True)
            target_angles = k.left_arm_set_position(angles, p, ori)
            a = [a for a in target_angles]
            motion.setAngles(k.left_arm_tags, a, 1.0)
            time.sleep(0.1)

def left_arm_ik_test(verbose=False):
    angle_set = [
        [0] * 5,
        k.left_arm_initial_pose,
        k.left_arm_work_pose]
    pos, ori = k.left_arm_get_position(angle_set[2])
    a =  angle_set[1]
    delta =  angle_set[2] - k.left_arm_set_position(a, pos,ori)
    print np.sum(delta*delta)
    

def right_arm_ik_test(verbose=False):
    angle_set = [
        [0] * 5,
        k.right_arm_initial_pose,
        k.right_arm_work_pose]
    pos, ori = k.right_arm_get_position(angle_set[2])
    a =  angle_set[1]
    
    delta = angle_set[2] - k.right_arm_set_position(a, pos,ori)
    print np.sum(delta*delta)
    
    
def left_arm_fk_test(verbose=False):
    motion = naoqi.ALProxy("ALMotion", host, port)
    angle_set  = [ 
        [0] * 5,
        [0, 0, -1.57, 0, 0],
        [1.57, 0, 0, 0, 0],
        [0, 0, 0, 0, -1.57],
        [0, 0, 0, -1.57, 0],
        [0, 0, -1.57, -1.57, 0],
        [0, 0, -1.57, -1.57, 0],
        [1.57, 1.57, 0, 0, 0],
        [1.57, 1.57, 0, 0, -1.57],
        [1.57, 1.57, 0, 0, 1.57],
        [1.57, 0, 0, 0, 0],
        ]
    for a in angle_set:
        motion.setAngles(k.left_arm_tags, a, 1.0)
        time.sleep(2.0)
        pos0 = motion.getPosition("LArm", 0, False)
        angles = motion.getAngles(k.left_arm_tags, False)
        pos_, ori_ = k.left_arm_get_position(angles)

        pos = pos0[0:3]
        pos[0] = pos[0] + 0.057
        pos[2] = pos[2] - 0.08682
        pos_error = [p - p_ for p, p_ in zip(pos, pos_)]
        sum_pos_error = 0
        for p in pos_error:
            sum_pos_error = sum_pos_error + math.fabs(p)

        ori = pos0[3:]
        Tx = k.transX(ori[0],0,0,0)
        Ty = k.transY(-ori[1],0,0,0)
        Tz = k.transZ(ori[2],0,0,0)
        T = Tz.dot(Ty.dot(Tx))[0:3,0:3]
        T_ = ori_

        ori_error = T - T_

        sum_ori_error = 0
        for o in ori_error:
            for v in o:
                sum_ori_error = sum_ori_error + math.fabs(v)
            pass
        print 'Error(a,pos ori),   ', a, ', ', sum_pos_error, ', ', sum_ori_error


def right_arm_fk_test():
    motion = naoqi.ALProxy("ALMotion", host, port)
    angle_set  = [ [0] * 5,
                   [1.57, 0, 0, 0, 0],
                   [0, 0, 1.57, 0, 0],
                   [1.57, 0, 0, 0, 0],
                   [0, 0, 0, 0, 1.57],
                   [0, 0, 0, 1.57, 0],
                   [0, 0, 1.57, 1.57, 0],
                   [1.57, -1.57, 0, 0, 0],
                   [1.57, -1.57, 0, 0, -1.57],
                   [1.57, -1.57, 0, 0, 1.57],
                   [1.57, 0, 0, 0, 0],
                   ]
    for a in angle_set:
        motion.setAngles(k.right_arm_tags, a, 1.0)
        time.sleep(2.0)
        pos0 = motion.getPosition("RArm", 0, True)
        angles = motion.getAngles(k.right_arm_tags, True)    
        pos_, ori_ = k.right_arm_get_position(angles)

        pos = pos0[0:3]
        pos[0] = pos[0] + 0.057
        pos[2] = pos[2] - 0.08682
        pos_error = [p - p_ for p, p_ in zip(pos, pos_)]
        sum_pos_error = 0
        for p in pos_error:
            sum_pos_error = sum_pos_error + math.fabs(p)

        ori = pos0[3:]
        Tx = k.transX(ori[0],0,0,0)
        Ty = k.transY(-ori[1],0,0,0)
        Tz = k.transZ(ori[2],0,0,0)
        T = Tz.dot(Ty.dot(Tx))[0:3,0:3]
        T_ = ori_

        ori_error = T - T_

        sum_ori_error = 0
        for o in ori_error:
            for v in o:
                sum_ori_error = sum_ori_error + math.fabs(v)
            pass
        print 'Error(a, pos ori),   ', a, ', ', sum_pos_error, ', ', sum_ori_error

if __name__ == '__main__':
    #left_arm_fk_test()
    #right_arm_fk_test()
    #left_arm_ik_test()
    #right_arm_ik_test()
    left_arm_ik_continuous()
