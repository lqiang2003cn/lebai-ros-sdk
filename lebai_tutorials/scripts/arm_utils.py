import copy
import math

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import translation_matrix, quaternion_matrix, euler_from_quaternion, translation_from_matrix, \
    quaternion_from_matrix

DE2RA = math.pi / 180
RA2DE = 180 / math.pi


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


def get_pos_and_quat_from_matrix(obj_mat):
    pos = translation_from_matrix(obj_mat)
    quat = quaternion_from_matrix(obj_mat)
    return pos, quat


def get_matrix_from_pos_and_quat(pos, quat):
    t_mat = translation_matrix(pos)
    q_mat = quaternion_matrix(quat)
    p_mat = np.dot(t_mat, q_mat)
    return p_mat


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
            print e
            try_times += 1
            continue
    return pos, rot


def query_pose_as_matrix(tf_listener, target_frame, source_frame):
    tf_listener.clear()
    pos, rot = None, None
    try_times = 0
    while not rospy.is_shutdown() and try_times < 100:
        try:
            tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4))
            pos, rot = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except Exception as e:
            print e
            try_times += 1
            continue
    return get_matrix_from_pos_and_quat(pos, rot)


def transform_pose(tf_listener, target_frame, pose):
    tf_listener.clear()
    transformed_pose = None
    while not rospy.is_shutdown():
        try:
            transformed_pose = tf_listener.transformPose(target_frame, pose)
            break
        except Exception as e:
            print e
            continue
    return transformed_pose.pose


def pose_by_diff(pose_mat, pos_diff, quat_diff):
    diff_mat = get_matrix_from_pos_and_quat(pos_diff, quat_diff)
    transformed_mat = np.dot(pose_mat, diff_mat)
    return transformed_mat


def get_rotation_angle(listener, obj_frame):
    # calculate everything in the world
    obj_pos, obj_ori_quat = query_pose(listener, "world", obj_frame)
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


def get_object_above_pose(listener, obj_pose_matrix, prepick_diff):
    m_new = np.eye(4, 4)
    obj_pos, obj_quat = get_pos_and_quat_from_matrix(obj_pose_matrix)

    bf_pos, bf_quat = query_pose(listener, "world", "world")
    bf_frame_mat = get_matrix_from_pos_and_quat(bf_pos, bf_quat)
    bf_x = bf_frame_mat[0:3, 0]
    bf_z = bf_frame_mat[0:3, 2]

    # up axis
    up_axis = -1
    abs_up_angle_min = np.inf
    up_direction = None
    for i in range(3):
        axis = obj_pose_matrix[0:3, i]
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
            axis = obj_pose_matrix[0:3, i]
            abs_front_angle = abs_angle_between(bf_x, axis)
            if abs_front_angle < abs_front_angle_min:
                abs_front_angle_min = abs_front_angle
                front_axis = i
                front_angle = angle_between(bf_x, axis)
                if front_angle <= math.pi / 2:
                    front_direction = 1
                else:
                    front_direction = -1

    m_new[0:3, 0] = obj_pose_matrix[0:3, front_axis] * front_direction
    m_new[0:3, 2] = -1 * obj_pose_matrix[0:3, up_axis] * up_direction
    m_new[0:3, 1] = np.cross(m_new[0:3, 2], m_new[0:3, 0])
    m_new_quat = quaternion_from_matrix(m_new)

    prepick_mat = get_matrix_from_pos_and_quat(obj_pos, m_new_quat)
    prepick_transform_mat = get_matrix_from_pos_and_quat(prepick_diff, [0, 0, 0, 1])
    prepick_mat = np.dot(prepick_mat, prepick_transform_mat)
    # prepick_pose_pos, prepick_pose_quat = get_pose_from_matrix(prepick_mat)

    return prepick_mat


def get_min_pose(listener, reference_axis, obj_frame):
    obj_pose_matrix = query_pose_as_matrix(listener, 'world', obj_frame)
    min_axis = -1
    min_abs_angle = np.inf
    min_direction = None
    for i in range(2):  # assuming z up
        axis = obj_pose_matrix[0:3, i]
        angle = abs_angle_between(reference_axis, axis)
        if angle < min_abs_angle:
            min_axis = i
            min_abs_angle = angle
            front_angle = angle_between(reference_axis, axis)
            if front_angle <= math.pi / 2:
                min_direction = 1
            else:
                min_direction = -1

    # specific to tool0's frame
    m_new = np.eye(4, 4)
    # y axis
    m_new[0:3, 1] = [0, 0, -1]
    # z axis
    m_new[0:3, 2] = obj_pose_matrix[0:3, min_axis] * min_direction
    # x axis
    m_new[0:3, 0] = np.cross(m_new[0:3, 1], m_new[0:3, 2])
    m_new_quat = quaternion_from_matrix(m_new)

    obj_position = translation_from_matrix(obj_pose_matrix)
    # obj_position[0] -= 0.02
    prepick_mat = get_matrix_from_pos_and_quat(obj_position, m_new_quat)

    obj_pos_diff = [0, -0.1, -0.18]
    prepick_transform_mat = get_matrix_from_pos_and_quat(obj_pos_diff, [0, 0, 0, 1])
    prepick_mat = np.matmul(prepick_mat, prepick_transform_mat)
    prepick_pos, prepick_ori = get_pos_and_quat_from_matrix(prepick_mat)
    return prepick_pos, prepick_ori, min_abs_angle


def get_circle_pose(listener, obj_frame):
    obj_pos, obj_quat = query_pose(listener, 'world', obj_frame)
    obj_pos_zero_z = copy.deepcopy(obj_pos)
    obj_pos_zero_z[2] = 0
    m_new = np.eye(4, 4)
    # y axis
    m_new[0:3, 1] = [0, 0, -1]
    # z axis
    m_new[0:3, 2] = unit_vector(obj_pos_zero_z)
    # x axis
    m_new[0:3, 0] = np.cross(m_new[0:3, 1], m_new[0:3, 2])
    prepick_quat = quaternion_from_matrix(m_new)

    prepick_mat = get_matrix_from_pos_and_quat(obj_pos, prepick_quat)
    obj_pos_diff = [0, -0.1, -0.18]
    prepick_transform_mat = get_matrix_from_pos_and_quat(obj_pos_diff, [0, 0, 0, 1])
    prepick_mat = np.matmul(prepick_mat, prepick_transform_mat)
    prepick_pos, prepick_ori = get_pos_and_quat_from_matrix(prepick_mat)
    return prepick_pos, prepick_ori


def get_circle_pose_by_pose_msg(pose_msg):
    obj_pos, obj_quat = get_pos_and_ori_from_pose_msg(pose_msg)
    obj_pos_zero_z = copy.deepcopy(obj_pos)
    obj_pos_zero_z[2] = 0
    m_new = np.eye(4, 4)
    # y axis
    m_new[0:3, 1] = [0, 0, -1]
    # z axis
    m_new[0:3, 2] = unit_vector(obj_pos_zero_z)
    # x axis
    m_new[0:3, 0] = np.cross(m_new[0:3, 1], m_new[0:3, 2])
    prepick_quat = quaternion_from_matrix(m_new)

    prepick_mat = get_matrix_from_pos_and_quat(obj_pos, prepick_quat)
    obj_pos_diff = [0, -0.1, -0.18]
    prepick_transform_mat = get_matrix_from_pos_and_quat(obj_pos_diff, [0, 0, 0, 1])
    prepick_mat = np.matmul(prepick_mat, prepick_transform_mat)
    prepick_pos, prepick_ori = get_pos_and_quat_from_matrix(prepick_mat)
    return prepick_pos, prepick_ori


def get_pose_msg_from_pos_and_ori(pos, ori):
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation.x = ori[0]
    pose.orientation.y = ori[1]
    pose.orientation.z = ori[2]
    pose.orientation.w = ori[3]
    return pose


def get_pos_and_ori_from_pose_msg(pose_msg):
    pos = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    ori = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
    return np.array(pos), np.array(ori)
