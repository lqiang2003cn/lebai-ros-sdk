#!/usr/bin/env python
# coding: utf-8
import math

import rospy
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from utils import get_current_angles, DE2RA, arm_joint_radian_to_degree, go_to_arm_joints

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("set_pose_py")
    # 初始化机械臂
    yahboomcar = MoveGroupCommander("arm")
    # 当运动规划失败后，允许重新规划
    yahboomcar.allow_replanning(True)
    yahboomcar.set_planning_time(10)
    # 尝试规划的次数
    yahboomcar.set_num_planning_attempts(10)
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    yahboomcar.set_goal_position_tolerance(0.005)
    yahboomcar.set_goal_orientation_tolerance(0.005)
    yahboomcar.set_goal_tolerance(0.005)
    # 设置允许目标误差
    print("end effector is:" + yahboomcar.get_end_effector_link())
    # 设置允许的最大速度和加速度
    yahboomcar.set_max_velocity_scaling_factor(1.0)
    yahboomcar.set_max_acceleration_scaling_factor(1.0)
    yahboomcar.set_pose_reference_frame("base_footprint")
    sleep(0.5)

    start_pose = yahboomcar.get_current_pose(yahboomcar.get_end_effector_link()).pose
    print("current position\n" + str(start_pose.position))
    print("current orientation\n" + str(start_pose.orientation))

    all_joints_d = get_current_angles()
    arm_link1_joint_r = all_joints_d[0] * DE2RA
    dist = -0.02
    x_inc = math.sin(arm_link1_joint_r) * dist
    y_inc = math.cos(arm_link1_joint_r) * dist
    print("arm link1 joint angle degree is:", all_joints_d[0])
    print("arm link1 joint angle radian is:", arm_link1_joint_r)
    print("x_inc is:", x_inc)
    print("y_inc is:", y_inc)

    pose = Pose()
    pose.position.x = start_pose.position.x + x_inc
    pose.position.y = start_pose.position.y + y_inc
    pose.position.z = start_pose.position.z

    q = start_pose.orientation
    pose.orientation.x = q.x
    pose.orientation.y = q.y
    pose.orientation.z = q.z
    pose.orientation.w = q.w
    yahboomcar.set_pose_target(pose)
    for i in range(3):
        plan = yahboomcar.plan()
        if len(plan.joint_trajectory.points) != 0:
            print("plan success")
            target_pos_radian = plan.joint_trajectory.points[-1].positions
            print("the target point radian is:\n", target_pos_radian)
            target_pos_degree = arm_joint_radian_to_degree(target_pos_radian)
            print("the target point degree is:\n", target_pos_degree)
            go_to_arm_joints(target_pos_degree)
            break
        else:
            print("plan error")
