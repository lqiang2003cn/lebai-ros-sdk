import moveit_commander
import rospy
from geometry_msgs.msg import PoseStamped
import arm_utils as autil


class LqPublish(object):

    def __init__(self):
        self.manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
        self.sleep_rate = rospy.Rate(1.0)
        self.pub_target_pose = rospy.Publisher("/target_pose", PoseStamped, queue_size=1)

    def start(self):
        rospy.loginfo("Publishing Information")
        while not rospy.is_shutdown():
            curr_pose = self.manipulator_group.get_current_pose().pose
            center = [0.00193624, -0.5609843, 0.07694705]
            target_position = [center[0], center[1], curr_pose.position.z]
            target_quant = autil.get_target_quant_from_obj_position(center)
            target_pose = autil.get_pose_stamped_msg_from_pos_and_ori("base_link", target_position, target_quant)
            self.pub_target_pose.publish(target_pose)
            self.sleep_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("lq_publish", anonymous=True)
    my_node = LqPublish()
    my_node.start()
