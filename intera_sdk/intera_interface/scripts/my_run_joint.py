#!/usr/bin/env python3

import rospy
import intera_interface

rospy.init_node('Hello_Sawyer')

limb = intera_interface.Limb('right')

limb.move_to_neutral()

angles = limb.joint_angles()

angles['right_j0']=0.0
angles['right_j1']=0.0
angles['right_j2']=0.0
angles['right_j3']=0.0
angles['right_j4']=0.0
angles['right_j5']=0.0
angles['right_j6']=0.0

limb.move_to_joint_positions(angles)

wave_1 = {'right_j6': -1.5126, 'right_j5': -0.3438, 'right_j4': 1.5126, 'right_j3': -1.3833, 'right_j2': 0.03726, 'right_j1': 0.3526, 'right_j0': -0.4259}

wave_2 = {'right_j6': -1.5101, 'right_j5': -0.3806, 'right_j4': 1.5103, 'right_j3': -1.4038, 'right_j2': -0.2609, 'right_j1': 0.3940, 'right_j0': -0.4281}

for _ in range(100):
    limb.move_to_joint_positions(wave_1)
    # rospy.sleep(0.5)
    limb.move_to_joint_positions(wave_2)
    # rospy.sleep(0.5)
