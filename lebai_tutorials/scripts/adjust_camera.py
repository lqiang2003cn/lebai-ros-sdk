import numpy as np
import rospy
import tf
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, translation_from_matrix
import arm_utils as autils
import aruco_msgs.msg as am


# 0.61025; -0.60255; 0.37153; 0.35567
# 0.19399; 0.10482; 0.70268
# 0.60588; -0.60057; 0.36725; 0.37062

def markers_callback(data):
    marker_pose = data.markers[0].pose.pose
    m_pos = marker_pose.position
    m_quat = marker_pose.orientation
    color_optical_to_aruco_pos = [m_pos.x, m_pos.y, m_pos.z]
    color_optical_to_aruco_ori = [m_quat.x, m_quat.y, m_quat.z, m_quat.w]
    marker_in_optical = autils.get_matrix_from_pos_and_quat(color_optical_to_aruco_pos, color_optical_to_aruco_ori)
    box_in_world = np.eye(4)
    box_in_world[0:3, 0] = [-1, 0, 0]
    box_in_world[0:3, 1] = [0, -1, 0]
    box_in_world[0:3, 2] = [0, 0, 1]
    # box_in_world[0:3, 3] = [-0.0315, -0.1148, 0.03]
    # box_in_world[0:3, 3] = [-0.0315, -0.1148, 0.03]
    box_in_world[0:3, 3] = [0.035, -0.2748, 0.03]
    color_optical_in_world = np.matmul(box_in_world, np.linalg.inv(marker_in_optical))

    # print np.concatenate([translation_from_matrix(color_optical_in_world), quaternion_from_matrix(color_optical_in_world)])

    optical_to_camera_pos = [-0.014, 0.000, -0.002]
    optical_to_camera_quat = [0.501, -0.500, 0.498, 0.501]
    optical_to_camera_matrix = autils.get_matrix_from_pos_and_quat(optical_to_camera_pos, optical_to_camera_quat)

    camera_in_world = np.matmul(color_optical_in_world, optical_to_camera_matrix)
    print np.concatenate([translation_from_matrix(camera_in_world), quaternion_from_matrix(camera_in_world)])


if __name__ == "__main__":
    rospy.init_node('get_camera_frame')
    listener = tf.TransformListener()

    # step1: run roslaunch (you might need to unplug one camera when you have multiple cameras)
    # roslaunch lebai_lm3_moveit_config gemini2_adjust_camera.launch

    # step2: adjust camera 01:
    # step2.1: put the 5cm aruco box to a position
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
    num_of_x_incr_blocks = 1
    num_of_y_incr_blocks = 0
    len_of_x = 0.068  # JBL
    len_of_y = 0.082  # JBL
    x_to_aruco_center = 0.03  # 5cm box
    y_to_aruco_center = 0.03  # 5cm box
    x_center = position1_x + len_of_x * num_of_x_incr_blocks + x_to_aruco_center
    y_center = position1_y - len_of_y * num_of_y_incr_blocks - y_to_aruco_center
    z_center = 0.105
    box_in_world[0:3, 3] = [x_center, y_center, z_center]
    color_optical_in_world = np.matmul(box_in_world, np.linalg.inv(marker_in_optical))
    # optical_to_camera_pos = autils.query_pose(listener, 'camera_color_optical_frame', 'camera_link')
    optical_to_camera_pos = [-0.014, 0.000, -0.002]
    optical_to_camera_quat = [0.501, -0.500, 0.498, 0.501]
    optical_to_camera_matrix = autils.get_matrix_from_pos_and_quat(optical_to_camera_pos, optical_to_camera_quat)
    camera_in_world = np.matmul(color_optical_in_world, optical_to_camera_matrix)
    print np.concatenate([translation_from_matrix(camera_in_world), quaternion_from_matrix(camera_in_world)])
    # step2.3: run the command in a terminal continuously to activate the aruco topic:
    # rostopic echo /cam01_aruco_single/marker
    # step2.4: run this node
    # get result, for example: 0.50033641 -0.63821168  0.38514528 -0.14800998  0.01318743  0.97093654 0.18762029

    # step4: put the result in the static_transform_publisher node in the marker_publisher.launch file


    marker_in_optical = autils.query_pose(listener, 'camera_color_optical_frame', 'aruco_marker_frame')
    color_optical_to_aruco_pos = marker_in_optical[0]
    color_optical_to_aruco_ori = marker_in_optical[1]
    marker_in_optical = autils.get_matrix_from_pos_and_quat(color_optical_to_aruco_pos, color_optical_to_aruco_ori)
    box_in_world = np.eye(4)
    box_in_world[0:3, 0] = [-1, 0, 0]
    box_in_world[0:3, 1] = [0, -1, 0]
    box_in_world[0:3, 2] = [0, 0, 1]
    position1_x = 0
    position1_y = -0.2348
    num_of_x_incr_blocks = -2
    num_of_y_incr_blocks = 0
    len_of_x = 0.068  # JBL
    len_of_y = 0.082  # JBL
    # x_to_aruco_center = 0.035 # JBL
    # y_to_aruco_center = 0.043 # JBL
    x_to_aruco_center = 0.03  # taller box
    y_to_aruco_center = 0.03  # taller
    x_center = position1_x + len_of_x * num_of_x_incr_blocks + x_to_aruco_center
    y_center = position1_y - len_of_y * num_of_y_incr_blocks - y_to_aruco_center
    z_center = 0.105
    box_in_world[0:3, 3] = [x_center, y_center, z_center]

    # camera 1 aruco
    # x: -0.0182075120141
    # y: -0.397250032065
    # z: 0.019383107582


    color_optical_in_world = np.matmul(box_in_world, np.linalg.inv(marker_in_optical))
    # optical_to_camera_pos = autils.query_pose(listener, 'camera_color_optical_frame', 'camera_link')
    optical_to_camera_pos = [-0.014, 0.000, -0.002]
    optical_to_camera_quat = [0.501, -0.500, 0.498, 0.501]
    optical_to_camera_matrix = autils.get_matrix_from_pos_and_quat(optical_to_camera_pos, optical_to_camera_quat)
    camera_in_world = np.matmul(color_optical_in_world, optical_to_camera_matrix)
    print np.concatenate([translation_from_matrix(camera_in_world), quaternion_from_matrix(camera_in_world)])

    print 'c'

# camera_link_to_color_optical_pos = [0.002, -0.014, -0.000]
# camera_link_to_color_optical_quat = [-0.501, 0.500, -0.498, 0.501]
# camera_link_to_color_optical_matrix = autils.get_matrix_from_pos_and_quat(camera_link_to_color_optical_pos,
#                                                                           camera_link_to_color_optical_quat)
#
# tmp = np.matmul(box_in_world, np.linalg.inv(marker_in_optical))
# camera_link_in_world = np.matmul(tmp, np.linalg.inv(camera_link_to_color_optical_matrix))
#
# print np.concatenate([translation_from_matrix(camera_link_in_world), quaternion_from_matrix(camera_link_in_world)])

# camera_to_aruco_pos = [0.21938, 0.11113, 0.687]
# camera_to_aruco_ori = [-0.57407, 0.63562, -0.39248, -0.33524]
# trans_matrix = autils.get_matrix_from_pos_and_quat(camera_to_aruco_pos, camera_to_aruco_ori)
# box_in_world = np.eye(4)
# box_in_world[0:3, 0] = [-1, 0, 0]
# box_in_world[0:3, 1] = [0, -1, 0]
# box_in_world[0:3, 2] = [0, 0, 1]
# box_in_world[0:3, 3] = [-0.0315, -0.1148, 0.03]
# camera_in_world = np.matmul(box_in_world, np.linalg.inv(trans_matrix))
# print camera_in_world
# print quaternion_from_matrix(camera_in_world)

# transform from camera link to aruco
# camera_to_aruco_pos = [0.00026535, 0.12414, 0.42982]
# camera_to_aruco_ori = [0.71423, 0.62391, 0.3113, -0.06081]
# trans_matrix = autils.get_matrix_from_pos_and_quat(camera_to_aruco_pos, camera_to_aruco_ori)
# box_in_world = np.eye(4)
# box_in_world[0:3, 0] = [0, 0, -1]
# box_in_world[0:3, 1] = [0, 1, 0]
# box_in_world[0:3, 2] = [1, 0, 0]
# box_in_world[0:3, 3] = [0.115, -0.467, 0.035]
# camera_in_world = np.matmul(box_in_world, np.linalg.inv(trans_matrix))
# print camera_in_world
# print quaternion_from_matrix(camera_in_world)

# aruco in world

# matrix_new = np.eye(4, 4)
# matrix_new[0:3, 0] = [0, 1, 0]
# matrix_new[0:3, 1] = [1, 0, 0]
# matrix_new[0:3, 2] = [0, 0, -1]
# print matrix_new
# print quaternion_from_matrix(matrix_new)

# matrix_new = np.eye(4, 4)
# matrix_new[0:3, 0] = [0, 1, 0]
# matrix_new[0:3, 1] = [0, 0, -1]
# matrix_new[0:3, 2] = [-1, 0, 0]
#
# print matrix_new
#
# # print quaternion_from_matrix(matrix_new)
#
# tool0_quaternion = [-0.45208, 0.89179, 0.010198, 0.015024]
# tool0_matrix = quaternion_matrix(tool0_quaternion)
# print tool0_matrix
#
# # tool0_euler = euler_from_quaternion(tool0_quaternion)
# # print tool0_euler
# box_quaternion = [0.08464, 0.69187, -0.071195, 0.7135]
# box_matrix = quaternion_matrix(box_quaternion)
# print box_matrix
#
# # x axis of tool0 equals z axis of box
# tool0_matrix[0:3, 0] = box_matrix[0:3, 2]
#
# # y axis of tool0  equals -y axis of box
# tool0_matrix[0:3, 1] = -box_matrix[0:3, 1]
#
# new_tool0_quaternion = quaternion_from_matrix(tool0_matrix)
# print new_tool0_quaternion
