#!/usr/bin/env python


import rospy
import rospkg
import math as m
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from point_cloud import array_to_xyz_pointcloud2f
import pygicp


class PointCloud(object):
    def __init__(self, pcd_topic):
        self.data = None

        self.slicing_point = None

        self.pcd_subscriber = rospy.Subscriber(
            pcd_topic, PointCloud2, callback=self.pointCloudCallback)

    def pointCloudCallback(self, msg):
        data = []

        for p in read_points(cloud=msg, field_names=("x", "y", "z"), skip_nans=True):
            data.append(p)

        self.data = data

    def slicePointCloud(self):
        if self.slicing_point is None:
            rospy.logwarn("NO SLICING POINT")
            return 0

        new_data = []

        for p in self.data:
            x = p[0]
            y = p[1]

            ix = self.slicing_point.pose.pose.position.x
            iy = self.slicing_point.pose.pose.position.y

            distance = m.sqrt((ix - x) ** 2 + (iy - y) ** 2)

            if distance < distance_threshold:
                new_data.append(p)

        rospy.loginfo("SET INITIAL POSE...")
        rospy.loginfo("SLICING MAP...")

        self.data = new_data


class GlobalPointCloud(PointCloud):
    def __init__(self, pcd_topic, point_topic):
        super(GlobalPointCloud, self).__init__(pcd_topic)

        self.initial_pose_subscriber = rospy.Subscriber(
            "initialpose", PoseWithCovarianceStamped, callback=self.initialPoseCallback
        )

    def initialPoseCallback(self, msg):
        if msg.header.frame_id != "map":
            rospy.logwarn("Initialpose topic frame is " +
                          str(msg.header.frame_id))
            rospy.logwarn("PLEASE CHANGE FIXED FRAME TO MAP IN RVIZ")
            return 0

        self.slicing_point = msg
        self.slicePointCloud()


class OdometrySubscriber(object):
    def __init__(self, ):

        self.position = None
        self.orientation = None

    def odometryCallback(self, msg):
        msg = Odometry()

        temp_position = msg.pose.pose.position
        temp_orientation = msg.pose.pose.orientation

        self.position = Position(
            x=temp_position.x, y=temp_position.y, z=temp_position.z)
        self.orientation = Orientation(
            x=temp_orientation.x, y=temp_orientation.y, z=temp_orientation.z, w=temp_orientation.w)


class Position(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Orientation(object):
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


def doMapMatching(target, source):

    if (target) is None or (source) is None:
        return np.array(
            [[0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 1]]
        )

    target = pygicp.downsample(target, 0.5)
    source = pygicp.downsample(source, 0.5)

    gicp = pygicp.FastGICP()
    gicp.set_input_target(target)
    gicp.set_input_source(source)

    gicp.set_num_threads(4)

    gicp.align()

    tf_matrix = gicp.get_final_transformation()

    return tf_matrix


def doTransform(source, tf_matrix):
    res = []

    for i in source:
        before_tf = np.array([i[0], i[1], i[2], 1])
        after_tf = np.dot(tf_matrix, before_tf)
        res.append([after_tf[0], after_tf[1], after_tf[2]])

    return res


if __name__ == "__main__":
    rospy.init_node("map_matching_localization")

    global_map_topic = rospy.get_param("global_map_topic", "globalmap")
    velodyne_topic = rospy.get_param("velodyne_topic", "velodyne_points")
    odom_topic = rospy.get_param("odom_topic", "/odometry/global")
    distance_threshold = int(rospy.get_param("map_matching_threshold", "30"))

    odom = OdometrySubscriber()

    point_cloud_publisher = rospy.Publisher(
        "align_test", PointCloud2, queue_size=5)

    odom_subscriber = rospy.Subscriber(
        odom_topic, Odometry, callback=odom.odometryCallback)

    global_map = GlobalPointCloud(pcd_topic=global_map_topic, point_topic=None)
    velodyne = PointCloud(pcd_topic=velodyne_topic)

    zero_point = PoseWithCovarianceStamped()
    zero_point.pose.pose.position.x = 0.0
    zero_point.pose.pose.position.y = 0.0

    velodyne.slicing_point = zero_point

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if (global_map.data) is not None and (velodyne.data) is not None:

            tf = doMapMatching(target=global_map.data, source=velodyne.data)
            res = doTransform(source=velodyne.data, tf_matrix=tf)

            print(tf)

            data = array_to_xyz_pointcloud2f(
                cloud_arr=res, stamp=rospy.Time.now(), frame_id="map")
            point_cloud_publisher.publish(data)

        rate.sleep()
