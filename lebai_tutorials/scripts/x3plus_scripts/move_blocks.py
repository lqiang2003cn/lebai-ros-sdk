#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
from copy import deepcopy

import moveit_commander
import numpy as np
import rospy
import sys
import tf
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

from utils import get_current_angles, DE2RA, arm_joint_radian_to_degree, go_to_arm_joints

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_plan_py')
    scene = PlanningSceneInterface()
    yahboomcar = MoveGroupCommander('arm')
    listener = tf.TransformListener()
    # 当运动规划失败后，允许重新规划
    yahboomcar.allow_replanning(True)
    yahboomcar.set_planning_time(50)
    yahboomcar.set_num_planning_attempts(20)
    yahboomcar.set_goal_position_tolerance(0.005)
    yahboomcar.set_goal_orientation_tolerance(0.005)
    yahboomcar.set_goal_tolerance(0.005)
    yahboomcar.set_max_velocity_scaling_factor(1.0)
    yahboomcar.set_max_acceleration_scaling_factor(1.0)
    yahboomcar.set_pose_reference_frame("base_footprint")

    # curr_joints = get_current_angles()
    # print("current joints:", curr_joints)
    #
    # joint_angle = get_current_angles()[0]
    # if joint_angle < 50 or joint_angle > 130:
    #     print("angle is outside of range")
    #     exit(0)
    all_joints_d = get_current_angles()
    arm_link1_joint_r = all_joints_d[0] * DE2RA
    dist = 0.04
    x_inc = math.sin(arm_link1_joint_r) * dist
    y_inc = math.cos(arm_link1_joint_r) * dist
    print("arm link1 joint angle degree is:", all_joints_d[0])
    print("arm link1 joint angle radian is:", arm_link1_joint_r)
    print("x_inc is:", x_inc)
    print("y_inc is:", y_inc)

    # 初始化路点列表
    end_effector_link = yahboomcar.get_end_effector_link()
    start_pose = yahboomcar.get_current_pose(end_effector_link).pose
    waypoints = []
    # 如果为True,将初始位姿加入路点列表
    waypoints.append(start_pose)
    wpose = deepcopy(start_pose)
    print("x is:", wpose.position.x)
    print("y is:", wpose.position.y)
    wpose.position.x += x_inc
    wpose.position.y += y_inc
    wpose.position.z = 0.035
    # wpose.orientation
    waypoints.append(deepcopy(wpose))
    # for i in range(3):
    #     # 设置路点数据，并加入路点列表
    #     wpose = deepcopy(start_pose)
    #     wpose.position.z += 0.13
    #     waypoints.append(deepcopy(wpose))
    #     wpose.position.z -= 0.13
    #     waypoints.append(deepcopy(wpose))
    # 规划过程
    fraction = 0.0  # 路径规划覆盖率
    maxtries = 100  # 最大尝试规划次数
    attempts = 0  # 已经尝试规划次数
    rospy.loginfo("Path Planning in Cartesian Space")
    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
    plan = None
    while fraction < 1.0 and attempts < maxtries:
        '''
        waypoints: 路点列表
        eef_step: 终端步进值，每隔0.1m计算一次逆解判断能否可达
        jump_threshold: 跳跃阈值，设置为0代表不允许跳跃
        plan: 路径, fraction: 路径规划覆盖率
        '''
        (plan, fraction) = yahboomcar.compute_cartesian_path(waypoints, 0.1, 0.0, True)
        attempts += 1
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the yahboomcar.")
        # print("the plan is:\n", plan)

        # grip_joint = all_joints[5]
        # print("grip joint:", grip_joint)
        # plan.joint_trajectory.joint_names.append("grip_joint")
        # for p in plan.joint_trajectory.points:
        #     lp = list(p.positions)
        #     lp.append(grip_joint)
        #     p.positions = tuple(lp)
        #     print(p.positions)
        # for p in plan.joint_trajectory.points:
        #     lp = list(p.velocities)
        #     lp.append(0.0)
        #     p.velocities = tuple(lp)
        #     print(p.velocities)
        # for p in plan.joint_trajectory.points:
        #     lp = list(p.accelerations)
        #     lp.append(0.0)
        #     p.accelerations = tuple(lp)
        #     print(p.accelerations)
        # print("plan joint list:", plan.joint_trajectory.joint_names)
        # print("the plan after modification is:\n", plan)

        # execute my plan by my own
        target_pos_radian = plan.joint_trajectory.points[-1].positions
        print("the target point radian is:\n", target_pos_radian)
        target_pos_degree = arm_joint_radian_to_degree(target_pos_radian)
        print("the target point degree is:\n", target_pos_degree)
        go_to_arm_joints(target_pos_degree)

        # yahboomcar.execute(plan, wait=True)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries))

    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()


