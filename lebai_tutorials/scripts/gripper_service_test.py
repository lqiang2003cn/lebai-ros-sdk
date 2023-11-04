#!/usr/bin/python
# coding=utf-8

import rospy

from lebai_msgs.srv import SetGripper

if __name__ == "__main__":
    rospy.init_node('create_ros_services')
    rospy.wait_for_service('/io_service/set_gripper_position')
    try:
        gripper_force_controller = rospy.ServiceProxy('/io_service/set_gripper_force', SetGripper)
        gripper_pos_controller = rospy.ServiceProxy('/io_service/set_gripper_position', SetGripper)
        print gripper_force_controller.call(5)
        print gripper_pos_controller.call(100.0)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
