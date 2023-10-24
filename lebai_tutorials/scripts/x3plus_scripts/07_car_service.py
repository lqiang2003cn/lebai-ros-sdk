#!/usr/bin/env python

from __future__ import print_function

import math
import numpy as np
from copy import deepcopy
from time import sleep
import moveit_commander
import rospy
import tf
from arm_moveit_demo.srv import Position, PositionResponse
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
from std_srvs.srv import Trigger, TriggerResponse

from utils import *

position_dict = {
    "initial position": [90, 145, 0, 45, 90],

    "front higher position": [90, 110, 75, 0, 90],
    "right higher position": [0, 110, 75, 0, 90],
    "left higher position": [180, 110, 75, 0, 90],

    "front lower position": [90, 150, 0, 36, 90],
    "right lower position": [0, 150, 0, 36, 90],
    "left lower position": [180, 150, 0, 36, 90],
    "camera down position higher": [50, 70, 20, 5, 90],
    # "camera down position higher": [70, 60, 0, 30, 90],
    "camera down position lower": [90, 50, 50, 17, 90],
    # "test position": [90, 25, 50, 17, 90] # 0.2617; -5.9061e-7; 0.05771   4.0591e-17; 0.99985; -3.6726e-6; 0.017452
    # "test position": [90, 10, 70, 17, 90] # 0.28494; -6.3542e-7; 0.045513 4.0591e-17; 0.99985; -3.6726e-6; 0.017452
    # "test position": [60, 20, 70, 17, 90] # 0.28494; -6.3542e-7; 0.045513 4.0591e-17; 0.99985; -3.6726e-6; 0.017452
    # "test position": [90.0, 9.0, 69.0, 16.0, 89.0, 30.0] # max x

    # min x: 0.24494; -5.9635e-7; 0.045514 -6.6455e-8; 0.99813; -3.6542e-6; 0.061049
    # "test position": [90.0, 40.0, 5.0, 48.0, 89.0]

    # min x: 0.21378; 0.066703; 0.03897
    # [-0.5235987755982988, -0.8726646259971648, -1.4835298641951802, -0.7330382858376184, -0.017453292519943295]
    "test position": [70.0, 40.0, 5.0, 48.0, 89.0]

}


def handle_go_to_position(position_msg):
    print("Going to position:", position_msg.position)
    try:
        pos_joints = position_dict[position_msg.position]
        curr_joints = get_current_angles()
        curr_joints[0:5] = pos_joints
        publish_joints(curr_joints)
    except KeyError as e:
        print("the position does not exists:", e)
        return PositionResponse(0)
    return PositionResponse(1)


def handle_close_gripper(trigger):
    assert trigger is not None
    print("Closing Gripper")
    try:
        curr_joints = get_current_angles()
        curr_joints[5] = 160
        publish_joints(curr_joints)
    except KeyError as e:
        print("the position does not exists:", e)
        return PositionResponse(0)

    return TriggerResponse()


def handle_open_gripper(trigger):
    assert trigger is not None
    print("Opening Gripper")
    try:
        curr_joints = get_current_angles()
        curr_joints[5] = 30
        publish_joints(curr_joints)
    except KeyError as e:
        print("the position does not exists:", e)
        return PositionResponse(0)

    return TriggerResponse()


def create_go_to_position_service():
    rospy.init_node('create_go_to_position_service')
    rospy.Service('go_to_position', Position, handle_go_to_position)
    rospy.Service('close_gripper', Trigger, handle_close_gripper)
    rospy.Service('open_gripper', Trigger, handle_open_gripper)
    print("Car services are ready")
    rospy.spin()


def go_to_position(position_info):
    print("Going to position:", position_info["position"])
    try:
        pos_joints = position_dict[position_info["position"]]
        curr_joints = get_current_angles()
        print("current joints:", curr_joints)
        curr_joints[0:5] = pos_joints
        publish_joints(curr_joints)
        sleep(2)
    except KeyError as e:
        print("the position does not exists:", e)
        return PositionResponse(0)
    return PositionResponse(1)


def go_to_position_by_arm_joints(arm_joints):
    print("Going to position:", arm_joints)
    try:
        curr_joints = get_current_angles()
        print("current joints:", curr_joints)
        curr_joints[0:5] = arm_joints
        publish_joints(curr_joints)
        sleep(2)
    except KeyError as e:
        print("the position does not exists:", e)


def open_gripper():
    print("Opening Gripper")
    try:
        curr_joints = get_current_angles()
        curr_joints[5] = 5
        publish_joints(curr_joints)
        sleep(2)
    except KeyError as e:
        print("open gripper failed", e)


def close_gripper():
    print("Closing Gripper")
    try:
        curr_joints = get_current_angles()
        curr_joints[5] = 160
        publish_joints(curr_joints)
        sleep(2)
    except KeyError as e:
        print("close gripper failed:", e)


def moveit_cartesian(tf_listener, frame_id):
    yahboomcar = MoveGroupCommander('arm')
    yahboomcar.allow_replanning(True)
    yahboomcar.set_planning_time(50)
    yahboomcar.set_num_planning_attempts(20)
    yahboomcar.set_goal_position_tolerance(0.02)
    yahboomcar.set_goal_orientation_tolerance(0.02)
    yahboomcar.set_goal_tolerance(0.02)
    yahboomcar.set_max_velocity_scaling_factor(1.0)
    yahboomcar.set_max_acceleration_scaling_factor(1.0)
    rospy.loginfo("Set Init Pose")
    sleep(0.5)

    end_effector_link = yahboomcar.get_end_effector_link()
    start_pose = yahboomcar.get_current_pose(end_effector_link).pose
    waypoints = [start_pose]
    for i in range(3):
        wpose = deepcopy(start_pose)
        wpose.position.z += 0.13
        waypoints.append(deepcopy(wpose))
        wpose.position.z -= 0.13
        waypoints.append(deepcopy(wpose))
    fraction = 0.0
    matrix = 100
    attempts = 0
    rospy.loginfo("Path Planning in Cartesian Space")
    plan = None
    while fraction < 1.0 and attempts < matrix:
        plan, fraction = yahboomcar.compute_cartesian_path(waypoints, 0.1, 0.0, True)
        attempts += 1
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the yahboomcar.")
        yahboomcar.execute(plan)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed with only " + str(
            fraction) + " success after " + str(matrix) + " attempts.")
    rospy.sleep(1)


def moveit_set_pose(tf_listener, frame_id):
    yahboomcar = MoveGroupCommander("arm")
    yahboomcar.allow_replanning(True)
    yahboomcar.set_planning_time(10)
    yahboomcar.set_num_planning_attempts(10)
    yahboomcar.set_goal_position_tolerance(0.003)
    yahboomcar.set_goal_orientation_tolerance(0.003)
    yahboomcar.set_goal_tolerance(0.003)
    print("end effector is:" + yahboomcar.get_end_effector_link())
    yahboomcar.set_max_velocity_scaling_factor(1.0)
    yahboomcar.set_max_acceleration_scaling_factor(1.0)
    yahboomcar.set_pose_reference_frame("base_footprint")
    sleep(0.5)

    current_pose = yahboomcar.get_current_pose(yahboomcar.get_end_effector_link()).pose
    print("current position\n" + str(current_pose.position))
    print("current orientation\n" + str(current_pose.orientation))

    # calculate arm_link1's angle and turn
    frame_pos_in_arm_link1, frame_ori_in_arm_link1 = query_pose(tf_listener, "arm_link1", frame_id)
    frame_pos_in_bf, frame_ori_in_bf = query_pose(tf_listener, "base_footprint", frame_id)
    print("frame position in arm link1 is :", frame_pos_in_arm_link1)
    print("frame position before rotate arm link1 is :", frame_pos_in_bf)
    frame_pos_in_arm_link1[2] = 0
    frame_pos = np.array(frame_pos_in_arm_link1)
    x_vec = np.array([0, 1, 0])
    angle_radian = angle_between(x_vec, frame_pos)
    print("aruco angle radian is:", angle_radian)
    angle_degree = RA2DE * angle_radian
    print("aruco angle degree is:", angle_degree)
    current_angles = get_current_angles()
    current_angles[0] = angle_degree
    publish_joints(current_angles)

    # adjust angle
    current_angles = get_current_angles()
    while np.abs(angle_degree - current_angles[0]) > 2:
        current_angles[0] = angle_degree
        publish_joints(current_angles)
        current_angles = get_current_angles()
        print("aruco angle:", angle_degree)
        print("published angle:", current_angles[0])

    # set xy
    frame_pos_in_arm_link5, frame_ori_in_arm_link5 = query_pose(tf_listener, "base_footprint", "arm_link5")
    frame_pos_in_bf, frame_ori_in_bf = query_pose(tf_listener, "base_footprint", frame_id)
    dist = get_euclidean(frame_pos_in_arm_link5[0:2], frame_pos_in_bf[0:2])
    print("dist is:", dist)
    arm_link1_joint_r = current_angles[0] * DE2RA
    x_inc = math.sin(arm_link1_joint_r) * dist
    y_inc = math.cos(arm_link1_joint_r) * dist
    print("x_inc is:", x_inc)
    print("y_inc is:", y_inc)

    end_effector_link = yahboomcar.get_end_effector_link()
    start_pose = yahboomcar.get_current_pose(end_effector_link).pose
    waypoints = [start_pose]
    wpose = deepcopy(start_pose)
    print("x is:", wpose.position.x)
    print("y is:", wpose.position.y)
    wpose.position.x += x_inc
    wpose.position.y += y_inc
    wpose.position.z = 0.035
    waypoints.append(deepcopy(wpose))

    fraction = 0.0
    maxtries = 100
    attempts = 0
    rospy.loginfo("Path Planning in Cartesian Space")
    plan = None
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = yahboomcar.compute_cartesian_path(waypoints, 0.1, 0.0, True)
        attempts += 1
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the yahboomcar.")
        target_pos_radian = plan.joint_trajectory.points[-1].positions
        print("the target point radian is:\n", target_pos_radian)
        target_pos_degree = arm_joint_radian_to_degree(target_pos_radian)
        print("the target point degree is:\n", target_pos_degree)
        go_to_arm_joints(target_pos_degree)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries))

    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()


def rotate_gripper(gripper_angle):
    print("Rotate Gripper")
    try:
        curr_joints = get_current_angles()
        curr_joints[4] = gripper_angle
        print("Before Rotate Gripper:", curr_joints)
        publish_joints(curr_joints)
        sleep(2)
    except KeyError as e:
        print("close gripper failed:", e)


if __name__ == "__main__":
    rospy.init_node('start_service')
    rate = rospy.Rate(10)
    listener = tf.TransformListener()

    # go to higher right
    higher_right_joints = [90, 70, 20, 3, 90]

    # lower right
    lower_right_joints = [90, 20, 0, 60, 90]

    # marker_to_search = "aruco_marker_frame"
    marker_to_search = "ar_marker_101"

    go_to_position_by_arm_joints(higher_right_joints)

    rot_angle = get_rotation_angle(listener, marker_to_search)
    rotate_gripper(rot_angle)

    # open_gripper()
    # close_gripper()
    # go_to_position_by_arm_joints(higher_right_joints)
    # moveit_set_pose(listener, marker_to_search)
    # close_gripper()
    # go_to_position_by_arm_joints(higher_right_joints)
    # open_gripper()
    # go_to_position_by_arm_joints(lower_right_joints)

    # while True:
    #     frame_pos_in_bf, frame_ori_in_bf = query_pose(listener, "base_footprint", marker_to_search)
    #     if frame_pos_in_bf is None or frame_ori_in_bf is None:
    #         continue
    #     else:
    #         print("block found")
    #         break

    # go to lower position
