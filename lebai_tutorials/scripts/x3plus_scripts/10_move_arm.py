#!/usr/bin/env python

from __future__ import print_function

from copy import copy
from math import pow, sqrt, atan2
import rospy
from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion, quaternion_matrix
from utils import get_current_angles, publish_joints, query_pose

if __name__ == "__main__":
    rospy.init_node('move_arm')
    listener = tf.TransformListener()

    # pos, rot = query_pose(listener, "base_footprint", "mono_link")
    # pos, rot = query_pose(listener, "mono_link", "ar_marker_107")

    pos, rot = query_pose(listener, "map", "base_footprint")
    print("pos:", pos)
    print("rot:", rot)
    print("rot euler:\n", euler_from_quaternion(rot))
    print("\n")

    pos, rot = query_pose(listener, "map", "ar_marker_107")
    print("pos:", pos)
    print("rot:", rot)
    print("rot matrix:\n", quaternion_matrix(rot))
    print("\n")

    pos, rot = query_pose(listener, "ar_marker_107", "base_footprint")
    print("pos:", pos)
    print("rot:", rot)
    print("rot matrix:\n", quaternion_matrix(rot))
    print("\n")

    # curr_joints = get_current_angles()
    # print("current joints", curr_joints)
    # curr_joints[1] = 135
    # curr_joints = [89.0, 170.0, 0.0, 9.0, 89.0, 30.0]
    # publish_joints(curr_joints)

