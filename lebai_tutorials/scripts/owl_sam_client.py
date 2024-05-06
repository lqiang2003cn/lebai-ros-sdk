#!/usr/bin/python
# coding=utf-8

from ctypes import *  # convert float to uint32

import geometry_msgs
import numpy
import numpy as np
import requests
import rospy
import sensor_msgs
import sensor_msgs.point_cloud2 as pc2
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
import ros_numpy as rnp

import utils


def default(obj):
    if type(obj).__module__ == np.__name__:
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return obj.item()
    raise TypeError('Unknown type:', type(obj))


class Nodo(object):

    def __init__(self):
        # Params
        self.cam01_rgb = None
        self.cam02_rgb = None
        self.cam01_pc = None
        self.cam02_pc = None
        self.depth = None
        self.texts = None
        self.br = CvBridge()
        self.sleep_rate = rospy.Rate(1.0)

        # Subscribers
        rospy.Subscriber("/ob_camera_01/color/image_raw", Image, self.cam01_rgb_callback)
        rospy.Subscriber("/ob_camera_02/color/image_raw", Image, self.cam02_rgb_callback)
        rospy.Subscriber("/ob_camera_01/depth_registered/points", PointCloud2, self.cam01_pc_callback)
        rospy.Subscriber("/ob_camera_02/depth_registered/points", PointCloud2, self.cam02_pc_callback)

    def cam01_rgb_callback(self, msg):
        self.cam01_rgb = self.br.imgmsg_to_cv2(msg)

    def cam02_rgb_callback(self, msg):
        self.cam02_rgb = self.br.imgmsg_to_cv2(msg)

    def cam01_pc_callback(self, msg):
        self.cam01_pc = msg

    def cam02_pc_callback(self, msg):
        self.cam02_pc = msg

    def request_owl_vit(self):
        while self.cam01_rgb is None:
            print "waiting for image"
            self.sleep_rate.sleep()

        while self.cam02_rgb is None:
            print "waiting for image"
            self.sleep_rate.sleep()

        while self.cam01_pc is None:
            print "waiting for pointcloud"
            self.sleep_rate.sleep()

        while self.cam02_pc is None:
            print "waiting for pointcloud"
            self.sleep_rate.sleep()

        cam01_pc_array = rnp.point_cloud2.pointcloud2_to_array(self.cam01_pc).reshape((480, 640))
        cam01_pc_points = rnp.point_cloud2.get_xyz_points(cam01_pc_array)
        cam01_pc_rgb_split = rnp.point_cloud2.split_rgb_field(cam01_pc_array)
        cam01_pc_rgb = np.zeros(cam01_pc_rgb_split.shape + (3,), dtype=np.int)
        cam01_pc_rgb[..., 0] = cam01_pc_rgb_split['r']
        cam01_pc_rgb[..., 1] = cam01_pc_rgb_split['g']
        cam01_pc_rgb[..., 2] = cam01_pc_rgb_split['b']

        # cam02 masked pc
        cam02_pc_array = rnp.point_cloud2.pointcloud2_to_array(self.cam02_pc).reshape((480, 640))
        cam02_pc_points = rnp.point_cloud2.get_xyz_points(cam02_pc_array)
        cam02_pc_rgb_split = rnp.point_cloud2.split_rgb_field(cam02_pc_array)
        cam02_pc_rgb = np.zeros(cam02_pc_rgb_split.shape + (3,), dtype=np.int)
        cam02_pc_rgb[..., 0] = cam02_pc_rgb_split['r']
        cam02_pc_rgb[..., 1] = cam02_pc_rgb_split['g']
        cam02_pc_rgb[..., 2] = cam02_pc_rgb_split['b']

        json_data = {
            "cameras": ["cam01", "cam02"],
            "images": [self.cam01_rgb.tolist(), self.cam02_rgb.tolist()],
            # "pc_points": [cam01_pc_points.tolist(), cam02_pc_points.tolist()],
            # "pc_rgbs": [cam01_pc_rgb.tolist(), cam02_pc_rgb.tolist()],
            "texts": [self.texts, self.texts]
        }
        response1 = utils.post_json_no_proxy("handle_all_json", json_data)
        print ''

        # get cam01 mask
        cam1_json_data = {
            "cam": "cam01",
            "image": self.cam01_rgb.tolist(),
            "texts": self.texts
        }
        response1 = utils.post_json_no_proxy("owl_vit", cam1_json_data)
        cam01_mask = np.array(response1.json()['mask'])[0]
        cam01_mask = np.load("masks_cam01.npy")[0]

        # get cam02 mask
        # cam2_json_data = {
        #     "cam": "cam02",
        #     "image": self.cam02_rgb.tolist(),
        #     "texts": self.texts
        # }
        # response2 = utils.post_json_no_proxy("owl_vit", cam2_json_data)
        # cam02_mask = np.array(response2.json()['mask'])[0]
        cam02_mask = np.load("masks_cam02.npy")[0]

        # cam01 masked pc
        cam01_pc_array = rnp.point_cloud2.pointcloud2_to_array(self.cam01_pc).reshape((480, 640))[cam01_mask]
        cam01_pc_xyz = rnp.point_cloud2.get_xyz_points(cam01_pc_array)
        cam01_pc_rgb_split = rnp.point_cloud2.split_rgb_field(cam01_pc_array)
        cam01_pc_rgb = np.zeros(cam01_pc_rgb_split.shape + (3,), dtype=np.int)
        cam01_pc_rgb[..., 0] = cam01_pc_rgb_split['r']
        cam01_pc_rgb[..., 1] = cam01_pc_rgb_split['g']
        cam01_pc_rgb[..., 2] = cam01_pc_rgb_split['b']

        # cam02 masked pc
        cam02_pc_array = rnp.point_cloud2.pointcloud2_to_array(self.cam02_pc).reshape((480, 640))[cam02_mask]
        cam02_pc_xyz = rnp.point_cloud2.get_xyz_points(cam02_pc_array)
        cam02_pc_rgb_split = rnp.point_cloud2.split_rgb_field(cam02_pc_array)
        cam02_pc_rgb = np.zeros(cam02_pc_rgb_split.shape + (3,), dtype=np.int)
        cam02_pc_rgb[..., 0] = cam02_pc_rgb_split['r']
        cam02_pc_rgb[..., 1] = cam02_pc_rgb_split['g']
        cam02_pc_rgb[..., 2] = cam02_pc_rgb_split['b']

        # merge cam01 and cam02
        all_xyz = np.concatenate([cam01_pc_xyz, cam02_pc_xyz], axis=0)
        np.save("all_xyz.npy", all_xyz)
        all_colors = np.concatenate([cam01_pc_rgb, cam02_pc_rgb], axis=0)
        np.save("all_colors.npy", all_colors)
        p = PointCloud()
        # p.header.stamp = rospy.Time.now()
        p.header.frame_id = "ob_camera_01_color_optical_frame"
        # p.points = xyz

        b = PointCloud()
        b.header.frame_id = "base_link"

        # p2 = PointCloud2()

        listener = tf.TransformListener()
        listener.clear()
        while not rospy.is_shutdown():
            try:
                listener.waitForTransform("base_link", "ob_camera_01_color_optical_frame", rospy.Time(), rospy.Duration(4))
                # translation,rotation = listener.lookupTransform(target_frame, hdr.frame_id, hdr.stamp)
                # fromTranslationRotation(translation, rotation)
                #
                #
                # listener.
                # r = sensor_msgs.msg.PointCloud2()
                # r.header.stamp = self.pointcloud2.header.stamp
                # r.header.frame_id = self.pointcloud2.header.frame_id
                # self.pointcloud2.header.frame_id = "/" + self.pointcloud2.header.frame_id
                # mat44 = listener.asMatrix("/base_link", self.pointcloud2.header)
                # def xf(p):
                #     xyz = tuple(numpy.dot(mat44, numpy.array([p.x, p.y, p.z, 1.0])))[:3]
                #     return geometry_msgs.msg.Point(*xyz)
                # r.points = [xf(p) for p in self.pointcloud2.points]

                # b = listener.asMatrix(p.header.frame_id, b.header)
                # np.save("matrix_cam_to_base.npy", b)
                # p = listener.transformPointCloud("base_link", p)
                break
            except Exception as e:
                self.sleep_rate.sleep()
        #     trans = tf_buffer.lookup_transform(
        #         "base_link",
        #         self.pointcloud2.header.frame_id,
        #         self.pointcloud2.header.stamp,
        #         rospy.Duration(1)
        #     )
        #     cloud_out = do_transform_cloud(self.pointcloud2, trans)

        # try:
        #     # masks_from_server = np.array(, dtype=np.bool)
        #     print("server response:" + response.json()["response"])
        # except requests.exceptions.RequestException:
        #     print(response.text)

    def start(self):
        rospy.loginfo("Timing images")
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            # br = CvBridge()
            if self.cam01_rgb is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.cam01_rgb))
            self.sleep_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("image_vis", anonymous=True)
    my_node = Nodo()
    # my_node.point_cloud_service()
    # my_node.texts = ["red block", "blue block", "yellow block", "green block"]
    my_node.texts = ["yellow cup"]
    my_node.request_owl_vit()
