#!/usr/bin/env python
from __future__ import print_function

import math
import numpy as np
from time import sleep

import rospy
from tf.transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix, translation_from_matrix, \
    euler_from_quaternion
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray, RobotArmArrayRequest

DE2RA = math.pi / 180
RA2DE = 180 / math.pi


def get_pos_and_quat_from_matrix(obj_mat):
    pos = translation_from_matrix(obj_mat)
    quat = quaternion_from_matrix(obj_mat)
    return pos, quat


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def abs_angle_between(v1, v2):
    angle = angle_between(v1, v2)
    if angle > math.pi / 2:
        angle = math.pi - angle
    return angle


def get_matrix_from_pos_and_quat(pos, quat):
    t_mat = translation_matrix(pos)
    q_mat = quaternion_matrix(quat)
    p_mat = np.dot(t_mat, q_mat)
    return p_mat


def get_rotation_angle(listener, obj_frame):
    obj_pos, obj_ori_quat = query_pose(listener, "base_footprint", obj_frame)
    obj_ori_rpy = euler_from_quaternion(obj_ori_quat)
    rot_angle = RA2DE * obj_ori_rpy[2]
    res = 90
    print("rot angle is:", rot_angle)
    if 5 < rot_angle < 85:
        res = 90 - rot_angle
    elif -85 < rot_angle < -5:
        res = 90 - rot_angle
    elif 95 < rot_angle < 175:
        rot_angle -= 90
        res = 90 - rot_angle
    elif -175 < rot_angle < -95:
        rot_angle += 90
        res = 90 - rot_angle
    return res

    # arm_link5_pos, arm_link5_ori_quat = query_pose(listener, "base_footprint", "arm_link5")
    # obj_mat = get_matrix_from_pos_and_quat(obj_pos, obj_ori_quat)
    # arm_link5_mat = get_matrix_from_pos_and_quat(arm_link5_pos, arm_link5_ori_quat)
    # print("object matrix:\n", obj_mat)
    # arm_link5_y = arm_link5_mat[0:3, 1]
    #
    # # y angle
    # abs_y_angle_min = np.inf
    # for i in range(2):
    #     axis = obj_mat[0:3, i]
    #     abs_y_angle_radian = abs_angle_between(arm_link5_y, axis)
    #     if abs_y_angle_radian < abs_y_angle_min:
    #         abs_y_angle_min = abs_y_angle_radian
    # print("c")


def get_object_above_pose(listener, obj_pose, prepick_diff):
    m_new = np.eye(4, 4)
    obj_pos, obj_quat = get_pos_and_quat_from_matrix(obj_pose)

    bf_pos, bf_quat = query_pose(listener, "odom", "base_footprint")
    bf_frame_mat = get_matrix_from_pos_and_quat(bf_pos, bf_quat)
    bf_x = bf_frame_mat[0:3, 0]
    bf_z = bf_frame_mat[0:3, 2]

    tube_frame_mat = get_matrix_from_pos_and_quat(obj_pos, obj_quat)

    # up axis
    up_axis = -1
    abs_up_angle_min = np.inf
    up_direction = None
    for i in range(3):
        axis = tube_frame_mat[0:3, i]
        abs_up_angle = abs_angle_between(bf_z, axis)
        if abs_up_angle < abs_up_angle_min:
            abs_up_angle_min = abs_up_angle
            up_axis = i
            up_angle = angle_between(bf_z, axis)
            if up_angle <= math.pi / 2:
                up_direction = 1
            else:
                up_direction = -1

    # front axis
    front_axis = -1
    abs_front_angle_min = np.inf
    front_direction = None
    for i in range(3):
        if i != up_axis:
            axis = tube_frame_mat[0:3, i]
            abs_front_angle = abs_angle_between(bf_x, axis)
            if abs_front_angle < abs_front_angle_min:
                abs_front_angle_min = abs_front_angle
                front_axis = i
                front_angle = angle_between(bf_x, axis)
                if front_angle <= math.pi / 2:
                    front_direction = 1
                else:
                    front_direction = -1

    m_new[0:3, 0] = tube_frame_mat[0:3, front_axis] * front_direction
    m_new[0:3, 2] = -1 * tube_frame_mat[0:3, up_axis] * up_direction
    m_new[0:3, 1] = np.cross(m_new[0:3, 2], m_new[0:3, 0])
    m_new_quat = quaternion_from_matrix(m_new)
    prepick_mat = get_matrix_from_pos_and_quat(obj_pos, m_new_quat)

    prepick_transform_mat = get_matrix_from_pos_and_quat(prepick_diff, [0, 0, 0, 1])
    prepick_mat = np.dot(prepick_mat, prepick_transform_mat)
    # prepick_pose_pos, prepick_pose_quat = get_pose_from_matrix(prepick_mat)

    return prepick_mat


def pose_by_diff(pose_mat, pos_diff, quat_diff):
    diff_mat = get_matrix_from_pos_and_quat(pos_diff, quat_diff)
    # pose_mat = get_matrix_from_pose(pose_pos, pose_quat)
    transformed_mat = np.dot(pose_mat, diff_mat)
    return transformed_mat


def center_to_tool(center_pose, center_to_tool_transform):
    res = np.dot(center_pose, center_to_tool_transform)
    return res


def get_euclidean(point1, point2):
    dist = np.linalg.norm(np.array(point1) - np.array(point2))
    return dist


def get_current_angles():
    curr_angle = rospy.ServiceProxy("CurrentAngle", RobotArmArray)
    request = RobotArmArrayRequest()
    response = curr_angle.call(request)
    return list(response.angles)


def publish_joints(joints):
    joints_pub = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
    arm_joint = ArmJoint()
    arm_joint.run_time = 5000
    arm_joint.joints = joints
    for i in range(10):
        joints_pub.publish(arm_joint)
        sleep(0.1)


def query_pose(tf_listener, target_frame, source_frame):
    tf_listener.clear()
    pos, rot = None, None
    try_times = 0
    while not rospy.is_shutdown() and try_times < 100:
        try:
            tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4))
            pos, rot = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except Exception as e:
            try_times += 1
            continue
    return pos, rot


def go_to_arm_joints(arm_joints):
    try:
        curr_joints = get_current_angles()
        print("current joints:", curr_joints)
        curr_joints[0:5] = arm_joints
        publish_joints(curr_joints)
    except KeyError as e:
        print("failed to go to position", e)


def arm_joint_degree_to_radian(arm_joint_degree):
    radian = list((np.array(arm_joint_degree) - 90) * DE2RA)
    return radian


def arm_joint_radian_to_degree(arm_joint_radian):
    degree = list(np.array(arm_joint_radian) * RA2DE + 90)
    return degree
