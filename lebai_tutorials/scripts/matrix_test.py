import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, translation_from_matrix
import arm_utils as autils

# 0.61025; -0.60255; 0.37153; 0.35567
# 0.19399; 0.10482; 0.70268
# 0.60588; -0.60057; 0.36725; 0.37062
color_optical_to_aruco_pos = [0.087, 0.016, 0.609]
color_optical_to_aruco_ori = [0.728, -0.555, 0.278, 0.291]
marker_in_optical = autils.get_matrix_from_pos_and_quat(color_optical_to_aruco_pos, color_optical_to_aruco_ori)
box_in_world = np.eye(4)
box_in_world[0:3, 0] = [-1, 0, 0]
box_in_world[0:3, 1] = [0, -1, 0]
box_in_world[0:3, 2] = [0, 0, 1]
# box_in_world[0:3, 3] = [-0.0315, -0.1148, 0.03]
# box_in_world[0:3, 3] = [-0.0315, -0.1148, 0.03]
box_in_world[0:3, 3] = [0.035, -0.2748, 0.03]
color_optical_in_world = np.matmul(box_in_world, np.linalg.inv(marker_in_optical))

print np.concatenate([translation_from_matrix(color_optical_in_world), quaternion_from_matrix(color_optical_in_world)])

optical_to_camera_pos = [-0.014, 0.000, -0.002]
optical_to_camera_quat = [0.501, -0.500, 0.498, 0.501]
optical_to_camera_matrix = autils.get_matrix_from_pos_and_quat(optical_to_camera_pos, optical_to_camera_quat)

camera_in_world = np.matmul(color_optical_in_world, optical_to_camera_matrix)
print np.concatenate([translation_from_matrix(camera_in_world), quaternion_from_matrix(camera_in_world)])

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
