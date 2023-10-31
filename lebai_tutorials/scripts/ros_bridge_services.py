#!/usr/bin/python
# coding=utf-8
import json

import rospy
from aruco_msgs.msg import MarkerArray
from lebai_tutorials.srv import JsonInfo


class RosBridgeServices:

    def __init__(self):
        rospy.init_node('create_ros_services')

        # init fields
        self.aruco_list = None
        self.aruco_mapping = {
            '200': '咖啡',
            '204': '矿泉水',
            '206': '牛奶'
        }

        # subscribes
        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.aruco_call_back)

        # creating services
        rospy.Service('get_goods_info', JsonInfo, self.get_goods_info)
        print 'services are initialized'

    def aruco_call_back(self, marker_array):
        self.aruco_list = marker_array.markers

    def get_goods_info(self, empty):
        assert empty is not None
        try:
            while self.aruco_list is None:
                rospy.sleep(0.5)  # waiting for aruco detector

            goods_info = []
            for al in self.aruco_list:
                goods_info.append({
                    'good_name': self.aruco_mapping[str(al.id)],
                    'good_no': 1,
                })
            return json.dumps(goods_info)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == "__main__":
    ros_services = RosBridgeServices()
    rospy.spin()
