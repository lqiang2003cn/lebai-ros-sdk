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

import utils

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2 ** 16
BIT_MOVE_8 = 2 ** 8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff))
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


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
        self.image = None
        self.pointcloud2 = None
        self.depth = None
        self.texts = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.sleep_rate = rospy.Rate(1.0)

        # call services
        # self.point_cloud_service = rospy.ServiceProxy('/camera/save_point_cloud', EmptyRequest)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image, queue_size=10)

        # Subscribers
        rospy.Subscriber("/ob_camera_01/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/ob_camera_01/depth_registered/points", PointCloud2, self.pointcloud2_callback)

    @staticmethod
    def convertCloudFromRosToOpen3d(ros_cloud):
        # Get cloud data from ros_cloud
        field_names = [field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

        # Check empty
        # open3d_cloud = open3d.PointCloud()
        if len(cloud_data) == 0:
            print("Converting an empty cloud")
            return None

        rgb = None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

            # Get xyz
            xyz = [Point32(x, y, z) for x, y, z, rgb in cloud_data]  # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:  # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]

            # combine
            # open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))
            # open3d_cloud.colors = open3d.Vector3dVector(np.array(rgb) / 255.0)
        else:
            xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
            # open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))

        # return
        return rgb, xyz

    def rgb_callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def pointcloud2_callback(self, msg):
        self.pointcloud2 = msg

    def request_owl_vit(self):
        while self.image is None:
            print "waiting for image"
            self.sleep_rate.sleep()

        while self.pointcloud2 is None:
            print "waiting for pointcloud"
            self.sleep_rate.sleep()

        json_data = {
            "image": self.image.tolist(),
            "texts": self.texts
        }

        rgb, xyz = Nodo.convertCloudFromRosToOpen3d(self.pointcloud2)

        p = PointCloud()
        # p.header.stamp = rospy.Time.now()
        p.header.frame_id = "ob_camera_01_color_optical_frame"
        p.points = xyz

        # p2 = PointCloud2()

        listener = tf.TransformListener()
        listener.clear()
        while not rospy.is_shutdown():
            try:
                listener.waitForTransform("base_link", "ob_camera_01_color_optical_frame", rospy.Time(), rospy.Duration(4))
            # r = sensor_msgs.msg.PointCloud2()
            # r.header.stamp = self.pointcloud2.header.stamp
            # r.header.frame_id = self.pointcloud2.header.frame_id
            # self.pointcloud2.header.frame_id = "/" + self.pointcloud2.header.frame_id
            # mat44 = listener.asMatrix("/base_link", self.pointcloud2.header)
            # def xf(p):
            #     xyz = tuple(numpy.dot(mat44, numpy.array([p.x, p.y, p.z, 1.0])))[:3]
            #     return geometry_msgs.msg.Point(*xyz)
            # r.points = [xf(p) for p in self.pointcloud2.points]
                p = listener.transformPointCloud("base_link", p)
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




        response = utils.post_json_no_proxy("owl_vit", json_data)
        try:
            # masks_from_server = np.array(, dtype=np.bool)
            print("server response:" + response.json()["response"])
        except requests.exceptions.RequestException:
            print(response.text)

    def start(self):
        rospy.loginfo("Timing images")
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            # br = CvBridge()
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image))
            self.sleep_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("image_vis", anonymous=True)
    my_node = Nodo()
    # my_node.point_cloud_service()
    # my_node.texts = ["red block", "blue block", "yellow block", "green block"]
    my_node.texts = ["yellow cup"]
    my_node.request_owl_vit()
