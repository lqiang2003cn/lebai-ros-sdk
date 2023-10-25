import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion
import arm_utils as autils

# transform from camera link to aruco
camera_to_aruco_pos = [0.00026535, 0.12414, 0.42982]
camera_to_aruco_ori = [0.71423, 0.62391, 0.3113, -0.06081]
trans_matrix = autils.get_matrix_from_pos_and_quat(camera_to_aruco_pos, camera_to_aruco_ori)

box_in_world = np.eye(4)
box_in_world[0:3, 0] = [0, 0, -1]
box_in_world[0:3, 1] = [0, 1, 0]
box_in_world[0:3, 2] = [1, 0, 0]
box_in_world[0:3, 3] = [0.115, -0.467, 0.035]

camera_in_world = np.matmul(box_in_world, np.linalg.inv(trans_matrix))
print camera_in_world
print quaternion_from_matrix(camera_in_world)
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
