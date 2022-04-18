#!/usr/bin/env python


import rospy
import math as m
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
import tf

distance_threshold = int(rospy.get_param("map_matching_threshold", "30"))


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


class OdometrySubscriber(object):
    def __init__(self, ):

        # This position and orientation data are belonged to ODOM frame

        self.position = None
        self.orientation = None

        self.tfListener = tf.TransformListener()

    def odometryCallback(self, msg):
        msg = Odometry()

        temp_position = msg.pose.pose.position
        temp_orientation = msg.pose.pose.orientation

        self.position = Position(
            x=temp_position.x, y=temp_position.y, z=temp_position.z)
        self.orientation = Orientation(
            x=temp_orientation.x, y=temp_orientation.y, z=temp_orientation.z, w=temp_orientation.w)

    def transformFrame(self):
        try:
            (trans, rot) = self.tfListener.lookupTransform(
                '/map', '/odom', rospy.Time(0))

            print(trans, rot)

        except Exception as ex:
            rospy.logwarn(ex)


class TF(object):
    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.pose = None

    def update(self, data):
        self.pose = data

    def broadcastTF(self):

        if self.pose is None:
            rospy.logwarn("NO TRANSFORM POSE")
            return 0

        p = self.pose.pose.pose.position
        ori = self.pose.pose.pose.orientation

        pose = [p.x, p.y, p.z]
        quat = [ori.x, ori.y, ori.z, ori.w]

        self.broadcaster.sendTransform(
            pose,
            quat,
            rospy.Time.now(),
            "odom",
            "map"
        )


class InitialPose(object):
    def __init__(self):
        self.data = None
        self.subscriber = rospy.Subscriber(
            "initialpose", PoseWithCovarianceStamped, callback=self.initialPoseCallback)

    def initialPoseCallback(self, msg):
        if msg.header.frame_id != "map":
            rospy.logwarn("Initialpose topic frame is " +
                          str(msg.header.frame_id))
            rospy.logwarn("PLEASE CHANGE FIXED FRAME TO MAP IN RVIZ")
            return 0

        rospy.loginfo("LISTENING INITIALPOSE...")
        self.data = msg
