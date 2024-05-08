import numpy as np
import rospy
import tf
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, translation_from_matrix
import arm_utils as autils
import aruco_msgs.msg as am


def adjust_camera_01():
    rospy.init_node('get_camera_frame')
    listener = tf.TransformListener()

    # step1: run roslaunch (you might need to unplug one camera when you have multiple cameras)
    # roslaunch lebai_lm3_moveit_config adjust_multi_cameras.launch

    # step2: adjust camera 01:
    # step2.1: put the 5cm aruco box to a position,
    # step2.2: adjust num_of_x_incr_blocks, num_of_y_incr_blocks based on the position of the 5cm box
    marker_in_optical = autils.query_pose(listener, 'ob_camera_01_color_optical_frame', 'cam01_aruco_marker_frame')
    color_optical_to_aruco_pos = marker_in_optical[0]
    color_optical_to_aruco_ori = marker_in_optical[1]
    marker_in_optical = autils.get_matrix_from_pos_and_quat(color_optical_to_aruco_pos, color_optical_to_aruco_ori)
    box_in_world = np.eye(4)
    box_in_world[0:3, 0] = [-1, 0, 0]
    box_in_world[0:3, 1] = [0, -1, 0]
    box_in_world[0:3, 2] = [0, 0, 1]
    position1_x = 0
    position1_y = -0.2348
    num_of_x_incr_blocks = 2  # if this is positive, put x-y corner to the right of the grid
    num_of_y_incr_blocks = 2
    len_of_x = 0.068  # JBL
    len_of_y = 0.082  # JBL
    x_to_aruco_center = 0.03 if num_of_x_incr_blocks >= 0 else -0.03
    y_to_aruco_center = 0.03  # 5cm box
    x_center = position1_x + len_of_x * num_of_x_incr_blocks + x_to_aruco_center
    y_center = position1_y - len_of_y * num_of_y_incr_blocks - y_to_aruco_center
    z_center = 0.105
    box_in_world[0:3, 3] = [x_center, y_center, z_center]
    color_optical_in_world = np.matmul(box_in_world, np.linalg.inv(marker_in_optical))
    optical_to_camera_pos = [-0.014, 0.000, -0.002]
    optical_to_camera_quat = [0.501, -0.500, 0.498, 0.501]
    optical_to_camera_matrix = autils.get_matrix_from_pos_and_quat(optical_to_camera_pos, optical_to_camera_quat)
    camera_in_world = np.matmul(color_optical_in_world, optical_to_camera_matrix)
    print np.concatenate([translation_from_matrix(camera_in_world), quaternion_from_matrix(camera_in_world)])
    # step2.3: run the command in a terminal continuously to activate the aruco topic:
    # rostopic echo /cam01_aruco_single/marker
    # step2.4: run this node
    # get result, for example: 0.465166 -0.42590476  0.37634892 -0.20534141 -0.02751783  0.97597023 -0.06752613


def adjust_camera_02():
    rospy.init_node('get_camera_frame')
    listener = tf.TransformListener()

    # step1: run roslaunch (you might need to unplug one camera when you have multiple cameras)
    # roslaunch lebai_lm3_moveit_config gemini2_adjust_camera.launch

    # step2: adjust camera 02:
    # step2.1: put the 5cm aruco box to a position
    # step2.2: adjust num_of_x_incr_blocks, num_of_y_incr_blocks based on the position of the 5cm box
    marker_in_optical = autils.query_pose(listener, 'ob_camera_02_color_optical_frame', 'cam02_aruco_marker_frame')
    color_optical_to_aruco_pos = marker_in_optical[0]
    color_optical_to_aruco_ori = marker_in_optical[1]
    marker_in_optical = autils.get_matrix_from_pos_and_quat(color_optical_to_aruco_pos, color_optical_to_aruco_ori)
    box_in_world = np.eye(4)
    box_in_world[0:3, 0] = [-1, 0, 0]
    box_in_world[0:3, 1] = [0, -1, 0]
    box_in_world[0:3, 2] = [0, 0, 1]
    position1_x = 0
    position1_y = -0.2348
    num_of_x_incr_blocks = -3  # if this is negative, put the -x-y corner to the left of the grid
    num_of_y_incr_blocks = 1
    len_of_x = 0.068  # JBL
    len_of_y = 0.082  # JBL
    x_to_aruco_center = 0.03 if num_of_x_incr_blocks >= 0 else -0.03  # if num_of_x_incr_blocks is negative, this should be negative
    y_to_aruco_center = 0.03  # 5cm box
    x_center = position1_x + len_of_x * num_of_x_incr_blocks + x_to_aruco_center
    y_center = position1_y - len_of_y * num_of_y_incr_blocks - y_to_aruco_center
    z_center = 0.105
    box_in_world[0:3, 3] = [x_center, y_center, z_center]
    color_optical_in_world = np.matmul(box_in_world, np.linalg.inv(marker_in_optical))
    optical_to_camera_pos = [-0.014, 0.000, -0.002]
    optical_to_camera_quat = [0.501, -0.500, 0.498, 0.501]
    optical_to_camera_matrix = autils.get_matrix_from_pos_and_quat(optical_to_camera_pos, optical_to_camera_quat)
    camera_in_world = np.matmul(color_optical_in_world, optical_to_camera_matrix)
    print np.concatenate([translation_from_matrix(camera_in_world), quaternion_from_matrix(camera_in_world)])
    # step2.3: run the command in a terminal continuously to activate the aruco topic:
    # rostopic echo /cam02_aruco_single/marker
    # step2.4: run this node
    # get result, for example: -0.54326354 -0.37640119  0.37693942 -0.00958675  0.23628084  0.01173544 0.97156664


if __name__ == "__main__":
    # adjust_camera_01()
    adjust_camera_02()
