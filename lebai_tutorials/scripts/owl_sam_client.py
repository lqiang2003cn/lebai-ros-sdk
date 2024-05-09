#!/usr/bin/python
# coding=utf-8

from ctypes import *  # convert float to uint32
import PIL.Image as PImage
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

        ################## Whole Steps:Start ##################
        # get transform masks
        target_frame = "base_link"
        cam_frames = ["ob_camera_01_color_optical_frame", "ob_camera_02_color_optical_frame"]
        listener = tf.TransformListener()
        listener.clear()
        base_to_cam_matrix_list = []
        for cf in cam_frames:
            while not rospy.is_shutdown():
                try:
                    listener.waitForTransform(target_frame, cf, rospy.Time(), rospy.Duration(4))
                    translation, rotation = listener.lookupTransform(target_frame, cf, rospy.Time(0))
                    base_to_cam_matrix_list.append(listener.fromTranslationRotation(translation, rotation))
                    break
                except Exception as e:
                    assert e
                    self.sleep_rate.sleep()

        json_data = {
            "cameras": ["cam01", "cam02"],
            "images": [self.cam01_rgb.tolist(), self.cam02_rgb.tolist()],
            # "pc_points": [cam01_pc_points.tolist(), cam02_pc_points.tolist()],
            # "pc_rgbs": [cam01_pc_rgb.tolist(), cam02_pc_rgb.tolist()],
            "texts": [self.texts, self.texts]
        }
        response = utils.post_json_no_proxy("handle_all_json", json_data)

        # get masked points: using mask to filter point cloud
        mask_list = response.json()['mask_list']
        pc_list = [self.cam01_pc, self.cam02_pc]
        filtered_points_list = []
        filtered_rgbs_list = []
        for i, m in enumerate(mask_list):
            m_np = np.array(m)[0]  # assuming there is only one mask?
            pc_array = rnp.point_cloud2.pointcloud2_to_array(pc_list[i]).reshape((480, 640))[m_np]
            filtered_points_list.append(rnp.point_cloud2.get_xyz_points(pc_array))
            pc_rgb_split = rnp.point_cloud2.split_rgb_field(pc_array)
            pc_rgbs = np.zeros(pc_rgb_split.shape + (3,), dtype=np.int)
            pc_rgbs[..., 0] = pc_rgb_split['r']
            pc_rgbs[..., 1] = pc_rgb_split['g']
            pc_rgbs[..., 2] = pc_rgb_split['b']
            filtered_rgbs_list.append(pc_rgbs)

        # transform masked points
        transformed_points_list = []
        for i in range(len(cam_frames)):
            filtered_points = filtered_points_list[i]
            filtered_points_extend = np.c_[filtered_points, np.ones(len(filtered_points))]
            base_to_cam_matrix = base_to_cam_matrix_list[i]
            transformed_points = np.einsum('ij,kj->ki', base_to_cam_matrix, filtered_points_extend)
            transformed_points_list.append(np.delete(transformed_points, 3, 1))

        # merge points: for two cameras
        all_points = np.concatenate([transformed_points_list[0], transformed_points_list[1]], axis=0)
        all_rgbs = np.concatenate([filtered_rgbs_list[0], filtered_rgbs_list[1]], axis=0)

        # center without removing outlier
        center = np.mean(all_points, axis=0)

        # Saving:
        # np.save("all_points.npy", all_points)
        # np.save("all_rgbs.npy", all_rgbs)
        # Load savings:
        # all_points = np.load("all_points.npy")
        # all_rgbs = np.load("all_rgbs.npy")

        # calculate center with outlier removal
        pc_json_data = {
            "all_points": all_points.tolist(),
            "all_rgbs": all_rgbs.tolist()
        }
        outlier_resp = utils.post_json_no_proxy("handle_outlier", pc_json_data)
        outlier_resp_json = outlier_resp.json()
        filtered_all_points_np = np.array(outlier_resp_json['filtered_all_points'])
        filtered_center = np.mean(filtered_all_points_np, axis=0)
        ################## Whole Steps:End ##################

        # todo: compare with aruco

        print 'filtered center' + str(filtered_center)
        return filtered_center

    def request_gpt4v(self):
        host = "http://localhost:8082/"
        method = "get_image_information"
        while self.cam01_rgb is None:
            print "waiting for image"
            self.sleep_rate.sleep()

        ################## Whole Steps:Start ##################
        image01_path = "/home/lq/lq_projects/gpt_4_vision/data/images/cam01_temp.jpg"
        pil_image = PImage.fromarray(self.cam01_rgb, "RGB")
        pil_image.save(image01_path)

        json_data = {
            "cameras": ["cam01"],
            "images_path": [image01_path],
        }
        response = utils.post_json_no_proxy(method, json_data, host=host)
        print response.json()

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

    # gpt4v image info
    my_node.request_gpt4v()

    # my_node.point_cloud_service()
    # my_node.texts = ["red block", "blue block", "yellow block", "green block"]

    # my_node.texts = ["yellow cup"]
    # cup_center = my_node.request_owl_vit()

    # moveit
