from lebai import LebaiRobot
import rospy
from joint_state_handler  import JointStateHandler
from io_state_handler  import IOStateHandler
from gripper_state_handler  import GripperStateHandler
from robot_state_handler  import RobotStateHandler
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
import urdf_helper
from urdf_parser_py.urdf import URDF


class RobotStateInterface:
    def __init__(self, ip):
        self.lebai_robot_ = LebaiRobot(ip, False)
        self.robot_description_param_ = rospy.get_param("/robot_description")
        self.urdf_robot_ = URDF.from_xml_string(self.robot_description_param_)
        self.joints_name_ = urdf_helper.find_chain_joints_name(self.urdf_robot_)
        self.joint_state_handler_ = JointStateHandler(self.lebai_robot_, self.joints_name_)
        self.io_state_handler_ = IOStateHandler(self.lebai_robot_)
        self.gripper_state_handler_ = GripperStateHandler(self.lebai_robot_)
        self.robot_state_handler_ = RobotStateHandler(self.lebai_robot_)
        

    def update_joint_state_handler(self):
        self.joint_state_handler_.run()

    def update_io_state_handler(self):
        self.io_state_handler_.run()        

    def update_gripper_state_handler(self):
        self.gripper_state_handler_.run()

    def update_robot_state_handler(self):
         self.robot_state_handler_.run()

    def run(self):
        self.update_joint_state_handler()
        self.update_io_state_handler()
        self.update_gripper_state_handler()
        self.update_robot_state_handler()
