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

        # subscribes
        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.aruco_call_back)

        # creating services
        rospy.Service('get_goods_info', JsonInfo, self.get_goods_info)
        rospy.Service('move_item_to_output_area', JsonInfo, self.move_item_to_output_area)

        print 'services are initialized'

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

    def move_item_to_output_area(self, params):
        assert params is not None
        try:
            json_param = json.loads(json.loads(params.input_json_str))
            item_id = json_param['item_id']

            # move to item's top position


            print 'item id is:' + str(json_param['item_id'])
            res = {'status': 'ok'}
            return json.dumps(res)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == "__main__":
    ros_services = RosBridgeServices()
    rospy.spin()
