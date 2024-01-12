#!/usr/bin/python
# coding=utf-8

import copy
import json
import sys

import geometry_msgs.msg as gm
import moveit_commander
import moveit_msgs.msg as mm
import numpy as np
import rospy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from lebai_tutorials.srv import JsonInfo
from moveit_msgs.msg import Constraints, OrientationConstraint
import arm_utils as autil
from lebai_msgs.srv import SetGripper

from tf.transformations import translation_matrix, quaternion_matrix, euler_from_quaternion, translation_from_matrix, \
    quaternion_from_matrix, quaternion_from_euler


class RosBridgeServices:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('create_ros_services')
        rospy.loginfo('starting ros services node')
        display_traj_topic = '/move_group/display_planned_path'

        # init moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher(display_traj_topic, mm.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()

        # init fields
        self.aruco_list = None

        # subscribes
        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.aruco_call_back)

        # creating services
        rospy.Service('get_goods_info', JsonInfo, self.get_goods_info)
        rospy.Service('move_item_to_output_area', JsonInfo, self.move_item_to_output_area)
        rospy.loginfo('services are initialized')

        # add scene objects
        self.add_objects_to_scene()

    def aruco_call_back(self, marker_array):
        self.aruco_list = marker_array.markers

    def get_goods_info(self, params):
        assert params is not None
        try:
            while self.aruco_list is None:
                rospy.sleep(0.5)  # waiting for aruco detector

            goods_info = []
            for al in self.aruco_list:
                goods_info.append({'good_id': al.id, 'good_no': 1})
            return json.dumps(goods_info)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def change_joint0(self, joint0_val):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = joint0_val
        p = self.move_group.plan(joint_goal)
        self.display_trajectory(p)
        self.execute_plan(p)
        self.move_group.stop()

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = mm.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    def add_objects_to_scene(self):
        rospy.sleep(1)
        self.scene.clear()
        # add objects to scene
        desk_pose = gm.PoseStamped()
        desk_pose.header.frame_id = "world"
        desk_pose.pose.orientation.w = 1.0
        desk_pose.pose.position.z = -0.1
        self.scene.add_box("desk", desk_pose, size=(1.4, 1.4, 0.25))
        self.scene.add_sphere('s', desk_pose, radius=0.075 / 2)
        print(self.move_group.get_end_effector_link())
        self.wait_for_state_update('desk', box_is_known=True, box_is_attached=False)

        link6_box_pose = gm.PoseStamped()
        link6_box_pose.header.frame_id = "link_6"
        link6_box_pose.pose.orientation.w = 1.0
        link6_box_pose.pose.position.y = -0.047
        link6_box_pose.pose.position.z = -0.025
        self.scene.add_box("link6_box", link6_box_pose, size=(0.01, 0.022, 0.01))
        self.wait_for_state_update('link6_box', box_is_known=True, box_is_attached=False)
        self.scene.attach_box('link_6', 'link6_box', touch_links=['link_6'])
        self.wait_for_state_update('link6_box', box_is_known=False, box_is_attached=True)

        gripper_box_pose = gm.PoseStamped()
        gripper_box_pose.header.frame_id = "gripper_base_link"
        gripper_box_pose.pose.orientation.w = 1.0
        gripper_box_pose.pose.position.x = -0.05
        gripper_box_pose.pose.position.z = 0.01
        self.scene.add_box("gripper_box", gripper_box_pose, size=(0.022, 0.01, 0.01))
        self.wait_for_state_update('gripper_box', box_is_known=True, box_is_attached=False)
        self.scene.attach_box('gripper_base_link', 'gripper_box', touch_links=['gripper_base_link'])
        self.wait_for_state_update('gripper_box', box_is_known=False, box_is_attached=True)
        rospy.loginfo('objects added to scene')

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

    def get_orientation_constraint(self, x, y, z):
        constraints = Constraints()
        constraints.name = "upright"
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = self.move_group.get_end_effector_link()
        current_pose = self.move_group.get_current_pose().pose
        orientation_constraint.orientation = current_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = x
        orientation_constraint.absolute_y_axis_tolerance = y
        orientation_constraint.absolute_z_axis_tolerance = z
        orientation_constraint.weight = 1
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def move_item_to_output_area(self, params):
        assert params is not None
        try:
            json_param = json.loads(json.loads(params.input_json_str))
            item_id = json_param['item_id']

            while self.aruco_list is None:
                rospy.sleep(0.5)  # waiting for aruco detector

            aruco_list = copy.deepcopy(self.aruco_list)
            item_pose_in_world = None
            for a in aruco_list:
                if a.id == item_id:
                    item_pose_in_world = a.pose.pose
            if item_pose_in_world is None:
                return {'status': 'wrong_item'}

            # move to item's top position
            ori_con = self.get_orientation_constraint(3.14, 0.01, 3.14)
            self.move_group.set_path_constraints(ori_con)
            prepick_pos, prepick_quat = autil.get_circle_pose_by_pose_msg(item_pose_in_world)
            prepick_pose = autil.get_pose_msg_from_pos_and_ori(prepick_pos, prepick_quat)
            is_plan_found = False
            waypoints = [copy.deepcopy(prepick_pose)]
            plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
                print 'constrained plan found'
                is_plan_found = True
                self.display_trajectory(plan)
            if is_plan_found:
                print 'executing plan'
                self.execute_plan(plan)
            self.move_group.stop()
            self.move_group.clear_path_constraints()
            self.move_group.clear_pose_targets()

            # move to item's front position
            ori_con = self.get_orientation_constraint(3.14, 0.01, 3.14)
            self.move_group.set_path_constraints(ori_con)
            current_pose = self.move_group.get_current_pose().pose
            pick_pose = copy.deepcopy(current_pose)
            pick_pose.position.z = 0.12
            is_plan_found = False
            waypoints = [copy.deepcopy(pick_pose)]
            plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
                print 'constrained plan found'
                is_plan_found = True
                self.display_trajectory(plan)
            if is_plan_found:
                print 'executing plan'
                self.execute_plan(plan)
            self.move_group.stop()
            self.move_group.clear_path_constraints()
            self.move_group.clear_pose_targets()

            # move to item's pick position
            ori_con = self.get_orientation_constraint(3.14, 0.01, 3.14)
            self.move_group.set_path_constraints(ori_con)
            current_pose = self.move_group.get_current_pose().pose
            prepick_mat = autil.get_matrix_from_pose_msg(current_pose)
            obj_pos_diff = [0, 0, 0.1]
            prepick_transform_mat = autil.get_matrix_from_pos_and_quat(obj_pos_diff, [0, 0, 0, 1])
            prepick_mat = np.matmul(prepick_mat, prepick_transform_mat)
            prepick_pose = autil.get_pose_msg_from_matrix(prepick_mat)
            is_plan_found = False
            waypoints = [copy.deepcopy(prepick_pose)]
            plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
                print 'constrained plan found'
                is_plan_found = True
                self.display_trajectory(plan)
            if is_plan_found:
                print 'executing plan'
                self.execute_plan(plan)
            self.move_group.stop()
            self.move_group.clear_path_constraints()
            self.move_group.clear_pose_targets()

            # close gripper
            rospy.wait_for_service('/io_service/set_gripper_position')
            try:
                gripper_force_controller = rospy.ServiceProxy('/io_service/set_gripper_force', SetGripper)
                gripper_pos_controller = rospy.ServiceProxy('/io_service/set_gripper_position', SetGripper)
                print gripper_force_controller.call(5)
                print gripper_pos_controller.call(0.0)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

            # move to top again
            ori_con = self.get_orientation_constraint(3.14, 0.01, 3.14)
            self.move_group.set_path_constraints(ori_con)
            prepick_pos, prepick_quat = autil.get_circle_pose_by_pose_msg(item_pose_in_world)
            prepick_pose = autil.get_pose_msg_from_pos_and_ori(prepick_pos, prepick_quat)
            is_plan_found = False
            waypoints = [copy.deepcopy(prepick_pose)]
            plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
                print 'constrained plan found'
                is_plan_found = True
                self.display_trajectory(plan)
            if is_plan_found:
                print 'executing plan'
                self.execute_plan(plan)
            self.move_group.stop()
            self.move_group.clear_path_constraints()
            self.move_group.clear_pose_targets()

            # move to output area: for now, fixed
            ori_con = self.get_orientation_constraint(3.14, 0.01, 3.14)
            self.move_group.set_path_constraints(ori_con)
            # fixed position
            current_pose = self.move_group.get_current_pose().pose
            prepick_pose = copy.deepcopy(current_pose)
            prepick_pose.position.x = -0.094
            prepick_pose.position.y = -0.358
            prepick_pose.orientation.x = 0.042
            prepick_pose.orientation.y = 0.718
            prepick_pose.orientation.z = -0.692
            prepick_pose.orientation.w = -0.057
            waypoints = [prepick_pose]
            is_plan_found = False
            plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
                print 'constrained plan found'
                is_plan_found = True
                self.display_trajectory(plan)
            if is_plan_found:
                print 'executing plan'
                self.execute_plan(plan)
            else:
                return json.dumps({"status": "plan failed"})
            self.move_group.stop()
            self.move_group.clear_path_constraints()
            self.move_group.clear_pose_targets()

            # move to place on table
            ori_con = self.get_orientation_constraint(3.14, 0.01, 3.14)
            self.move_group.set_path_constraints(ori_con)
            current_pose = self.move_group.get_current_pose().pose
            pick_pose = copy.deepcopy(current_pose)
            pick_pose.position.z -= 0.08
            is_plan_found = False
            waypoints = [copy.deepcopy(pick_pose)]
            plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction == 1.0 and len(plan.joint_trajectory.points) >= 2:
                print 'constrained plan found'
                is_plan_found = True
                self.display_trajectory(plan)
            if is_plan_found:
                print 'executing plan'
                self.execute_plan(plan)
            self.move_group.stop()
            self.move_group.clear_path_constraints()
            self.move_group.clear_pose_targets()

            # open gripper
            gripper_force_controller = rospy.ServiceProxy('/io_service/set_gripper_force', SetGripper)
            gripper_pos_controller = rospy.ServiceProxy('/io_service/set_gripper_position', SetGripper)
            print gripper_force_controller.call(5)
            print gripper_pos_controller.call(100.0)

            res = {'status': 'ok'}
            return json.dumps(res)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def make_transforms(self, parent_frame_id, child_frame_id, transform_pose, rate):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time(0)
        static_transformStamped.header.frame_id = parent_frame_id
        static_transformStamped.child_frame_id = child_frame_id
        static_transformStamped.transform.translation.x = transform_pose.position.x
        static_transformStamped.transform.translation.y = transform_pose.position.y
        static_transformStamped.transform.translation.z = transform_pose.position.z
        static_transformStamped.transform.rotation.x = transform_pose.orientation[0]
        static_transformStamped.transform.rotation.y = transform_pose.orientation[1]
        static_transformStamped.transform.rotation.z = transform_pose.orientation[2]
        static_transformStamped.transform.rotation.w = transform_pose.orientation[3]
        rate = rospy.Rate(10)
        tf_publisher = StaticTransformBroadcaster()
        while True:
            tf_publisher.sendTransform(static_transformStamped)
            rate.sleep()


if __name__ == "__main__":
    ros_services = RosBridgeServices()
    rospy.spin()
