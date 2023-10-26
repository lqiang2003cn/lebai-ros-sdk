import numpy as np
import rospy
from tf.transformations import translation_matrix, quaternion_matrix


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


def get_matrix_from_pos_and_quat(pos, quat):
    t_mat = translation_matrix(pos)
    q_mat = quaternion_matrix(quat)
    p_mat = np.dot(t_mat, q_mat)
    return p_mat
