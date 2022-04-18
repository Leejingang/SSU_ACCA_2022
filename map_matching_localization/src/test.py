#!/usr/bin/env python


import rospy
import rospkg
import math as m
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from point_cloud import array_to_xyz_pointcloud2f
from header import *
import pygicp


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

    global_map_topic = rospy.get_param("global_map_topic", "cloud_pcd")
    velodyne_topic = rospy.get_param("velodyne_topic", "velodyne_points")
    odom_topic = rospy.get_param("odom_topic", "/odometry/global")

    odom = OdometrySubscriber()
    init_pose = InitialPose()
    my_tf = TF()

    point_cloud_publisher = rospy.Publisher(
        "align_test", PointCloud2, queue_size=5)

    odom_subscriber = rospy.Subscriber(
        odom_topic, Odometry, callback=odom.odometryCallback)

    global_map = PointCloud(pcd_topic=global_map_topic)
    velodyne = PointCloud(pcd_topic=velodyne_topic)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        my_tf.update(init_pose.data)
        my_tf.broadcastTF()

        odom.transformFrame()

        rate.sleep()
