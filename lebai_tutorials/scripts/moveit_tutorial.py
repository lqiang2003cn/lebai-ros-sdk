#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import copy
import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import TrajectoryConstraints
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander.conversions import pose_to_list
from tf import broadcaster
from tf.transformations import quaternion_matrix, quaternion_from_matrix

import arm_utils as autil
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import Quaternion
import moveit_commander


def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame
        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] += 0.1
        p = move_group.plan(joint_goal)
        self.display_trajectory(p)
        self.execute_plan(p)
        move_group.stop()

    def go_to_pose_goal(self):
        move_group = self.move_group
        curr_pose = move_group.get_current_pose().pose

        # set to horizontal
        pose_goal = geometry_msgs.msg.Pose()
        pose_ori = np.eye(4)
        pose_ori[0:3, 0] = [0, 0, 1]
        pose_ori[0:3, 1] = [-1, 0, 0]
        pose_ori[0:3, 2] = [0, -1, 0]
        pose_ori_quat = quaternion_from_matrix(pose_ori)
        pose_goal.position.x = curr_pose.position.x
        pose_goal.position.y = curr_pose.position.y
        pose_goal.position.z = curr_pose.position.z
        pose_goal.orientation.x = pose_ori_quat[0]
        pose_goal.orientation.y = pose_ori_quat[1]
        pose_goal.orientation.z = pose_ori_quat[2]
        pose_goal.orientation.w = pose_ori_quat[3]

        move_group.set_pose_target(pose_goal)
        plan = move_group.plan()
        self.display_trajectory(plan)
        self.execute_plan(plan)
        move_group.stop()
        move_group.clear_pose_targets()

    @staticmethod
    def transform_to_tool0():
        # get pose from tf
        listener = tf.TransformListener()
        # box_pos, box_rot = autil.query_pose(listener, 'world', 'aruco_marker_frame')
        tool0_pos, tool0_ori = autil.query_pose(listener, 'world', 'tool0')

        # table_size = [0.1, 0.2, 0.02]
        # table_pose = PoseStamped()
        # table_pose.header.frame_id = 'base_link'
        # table_pose.pose.position.x = 0.2
        # table_pose.pose.position.y = 0
        # table_pose.pose.position.z = 0.2 + table_size[2] / 2.0
        # table_pose.pose.orientation.w = 1.0
        # scene.add_box('obj', table_pose, table_size)

        ps = PoseStamped()
        ps.header.frame_id = 'aruco_marker_frame'
        ps.pose.position.x = 0
        ps.pose.position.y = 0
        ps.pose.position.z = 0.05 + 0.2
        ps.pose.orientation.x = 0
        ps.pose.orientation.y = 0
        ps.pose.orientation.z = 0
        ps.pose.orientation.w = 1
        box_above_pose = autil.transform_pose(listener, 'world', ps)

        # handle rotation:
        box_matrix = quaternion_matrix([box_above_pose.orientation.x, box_above_pose.orientation.y, box_above_pose.orientation.z,
                                        box_above_pose.orientation.w])
        tool0_matrix = quaternion_matrix(tool0_ori)

        # x axis of tool0 equals -x axis of box
        tool0_matrix[0:3, 0] = -box_matrix[0:3, 0]

        # y axis of tool0  equals y axis of box
        tool0_matrix[0:3, 1] = box_matrix[0:3, 1]

        new_tool0_quaternion = quaternion_from_matrix(tool0_matrix)

        new_tool0_pose = Pose()
        new_tool0_pose.orientation.x = new_tool0_quaternion[0]
        new_tool0_pose.orientation.y = new_tool0_quaternion[1]
        new_tool0_pose.orientation.z = new_tool0_quaternion[2]
        new_tool0_pose.orientation.w = new_tool0_quaternion[3]
        new_tool0_pose.position = box_above_pose.position
        return new_tool0_pose

        # return box_above_pose

        # r = rospy.Rate(10)
        # while True:
        #     bc = tf.TransformBroadcaster()
        #     acc_pos = [box_above_pose.position.x, box_above_pose.position.y, box_above_pose.position.z]
        #     acc_ori = [box_above_pose.orientation.x, box_above_pose.orientation.y, box_above_pose.orientation.z, box_above_pose.orientation.w]
        #     bc.sendTransform(acc_pos, acc_ori, rospy.Time().now(), 'aruco_center_code', 'world')
        #     r.sleep()

        # tool0_quaternion = rot
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

    def plan_cartesian_path(self):
        move_group = self.move_group
        box_above_pose = MoveGroupPythonIntefaceTutorial.transform_to_tool0()
        waypoints = []
        # wpose = move_group.get_current_pose().pose

        # wpose.position.x += scale * 0.05
        waypoints.append(copy.deepcopy(box_above_pose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=100000000):
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_leftfinger"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07  # slightly above the end effector
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names
        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
    try:
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.scene.clear()
        listener = tf.TransformListener()

        # add objects to scene
        desk_pose = geometry_msgs.msg.PoseStamped()
        desk_pose.header.frame_id = "world"
        desk_pose.pose.orientation.w = 1.0
        desk_pose.pose.position.z = -0.1
        tutorial.scene.add_box("desk", desk_pose, size=(1.4, 1.4, 0.25))
        tutorial.wait_for_state_update('desk', box_is_known=True, box_is_attached=False)

        link6_box_pose = geometry_msgs.msg.PoseStamped()
        link6_box_pose.header.frame_id = "link_6"
        link6_box_pose.pose.orientation.w = 1.0
        link6_box_pose.pose.position.y = -0.047
        link6_box_pose.pose.position.z = -0.025
        tutorial.scene.add_box("link6_box", link6_box_pose, size=(0.01, 0.022, 0.01))
        tutorial.wait_for_state_update('link6_box', box_is_known=True, box_is_attached=False)
        tutorial.scene.attach_box('link_6', 'link6_box', touch_links=['link_6'])
        tutorial.wait_for_state_update('link6_box', box_is_known=False, box_is_attached=True)

        gripper_box_pose = geometry_msgs.msg.PoseStamped()
        gripper_box_pose.header.frame_id = "gripper_base_link"
        gripper_box_pose.pose.orientation.w = 1.0
        gripper_box_pose.pose.position.x = -0.05
        gripper_box_pose.pose.position.z = 0.01
        tutorial.scene.add_box("gripper_box", gripper_box_pose, size=(0.022, 0.01, 0.01))
        tutorial.wait_for_state_update('gripper_box', box_is_known=True, box_is_attached=False)
        tutorial.scene.attach_box('gripper_base_link', 'gripper_box', touch_links=['gripper_base_link'])
        tutorial.wait_for_state_update('gripper_box', box_is_known=False, box_is_attached=True)



        ###### use only world -y axis ######
        # aruco_frame = 'aruco_marker_frame'
        # world_my_pos, world_my_ori, world_my_min_angle = autil.get_min_pose(listener, [0, -1, 0], aruco_frame)
        # print 'world -y axis angle:' + str(world_my_min_angle)
        # is_plan_found = False
        # prepick_pose = autil.get_pose_msg_from_pos_and_ori(world_my_pos, world_my_ori)
        # waypoints = [copy.deepcopy(prepick_pose)]
        # plan, fraction = tutorial.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        # if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
        #     print 'better plan found'
        #     is_plan_found = True
        #     tutorial.display_trajectory(plan)


        ########### use both world -y axis and tool0 z axis ############
        # aruco_frame = 'aruco_marker_frame'
        # listener = tf.TransformListener()
        # # world -y axis plan
        # world_my_pos, world_my_ori, world_my_min_angle = autil.get_min_pose(listener, [0, -1, 0], aruco_frame)
        # print 'world -y axis angle:' + str(world_my_min_angle)
        #
        # # tool0 z axis plan
        # tool0_z_axis = autil.query_pose_as_matrix(listener, 'world', 'tool0')[0:3, 2]
        # tool0_z_pos, tool0_z_ori, tool0_z_min_angle = autil.get_min_pose(listener, tool0_z_axis, aruco_frame)
        # print 'tool0 z axis angle:' + str(tool0_z_min_angle)
        #
        # # smaller angle
        # if world_my_min_angle < tool0_z_min_angle:
        #     better_pos, better_ori = world_my_pos, world_my_ori
        #     worse_pos, worse_ori = tool0_z_pos, tool0_z_ori
        #     print 'smaller angle is: world -y axis angle'
        # else:
        #     better_pos, better_ori = tool0_z_pos, tool0_z_ori
        #     worse_pos, worse_ori = world_my_pos, world_my_ori
        #     print 'smaller angle is: tool0 z axis angle'
        #
        # # try better plan first
        # is_plan_found = False
        # prepick_pose = autil.get_pose_msg_from_pos_and_ori(better_pos, better_ori)
        # waypoints = [copy.deepcopy(prepick_pose)]
        # plan, fraction = tutorial.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        # if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
        #     print 'better plan found'
        #     is_plan_found = True
        #     tutorial.display_trajectory(plan)
        # else:
        #     # try worse plan
        #     prepick_pose = autil.get_pose_msg_from_pos_and_ori(worse_pos, worse_ori)
        #     waypoints = [copy.deepcopy(prepick_pose)]
        #     plan, fraction = tutorial.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #     if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
        #         print 'worse plan found'
        #         is_plan_found = True
        #         tutorial.display_trajectory(plan)
        #     else:
        #         print 'no plan found'
        #
        # if is_plan_found:
        #     print 'executing plan'
        #     tutorial.execute_plan(plan)
        #
        # # move to lower position
        # is_plan_found = False
        # pick_pose = copy.deepcopy(prepick_pose)
        # pick_pose.position.z = 0.08
        # waypoints = [copy.deepcopy(pick_pose)]
        #
        # # Create an OrientationConstraint
        # oc = OrientationConstraint()
        # oc.link_name = "tool0"
        # oc.header.frame_id = "world"
        # oc.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # specify the desired orientation here
        # oc.absolute_x_axis_tolerance = 0.1
        # oc.absolute_y_axis_tolerance = 0.1
        # oc.absolute_z_axis_tolerance = 0.1
        # oc.weight = 1.0
        # constraints = moveit_commander.Constraints()
        # constraints.orientation_constraints.append(oc)
        # tutorial.move_group.set_path_constraints(constraints)
        #
        # # plan, fraction = tutorial.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #
        # plan, fraction = tutorial.move_group.plan()
        # if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
        #     print 'pick plan found'
        #     is_plan_found = True
        #     tutorial.display_trajectory(plan)
        #
        # if is_plan_found:
        #     tutorial.execute_plan(plan)
        #     print 'c'

        # cartesian_plan, fraction = tutorial.plan_cartesian_path()
        # tutorial.go_to_joint_state()
        # tutorial.go_to_pose_goal()

        # raw_input()
        # tutorial.go_to_pose_goal()
        #
        # print "============ Press `Enter` to plan and display a Cartesian path ..."
        # raw_input()
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()
        #
        # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # raw_input()
        # tutorial.display_trajectory(cartesian_plan)
        #
        # print "============ Press `Enter` to execute a saved path ..."
        # raw_input()
        # tutorial.execute_plan(cartesian_plan)
        #
        # print "============ Press `Enter` to add a box to the planning scene ..."
        # raw_input()
        # tutorial.add_box()
        #
        # print "============ Press `Enter` to attach a Box to the Panda robot ..."
        # raw_input()
        # tutorial.attach_box()
        #
        # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # raw_input()
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)
        #
        # print "============ Press `Enter` to detach the box from the Panda robot ..."
        # raw_input()
        # tutorial.detach_box()
        #
        # print "============ Press `Enter` to remove the box from the planning scene ..."
        # raw_input()
        # tutorial.remove_box()
        #
        # print "============ Python tutorial demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
