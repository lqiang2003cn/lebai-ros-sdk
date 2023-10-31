#!/usr/bin/python

import rospy
from std_srvs.srv import Empty


def open_gripper(trigger):
    assert trigger is not None
    try:
        print 'calling ros open gripper service'
        return {}
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    rospy.init_node('create_lebai_services')
    rospy.Service('open_gripper', Empty, open_gripper)
    print 'services are initialized'
    rospy.spin()
