#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class PointCloudProfiler:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.sub = rospy.Subscriber(topic_name, PointCloud2, self.pc_callback)

    def pc_callback(self, msg):
        # Extract timestamp
        timestamp = msg.header.stamp.to_sec()

        # Extract XYZ coordinates and convert to numpy array
        points_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_arr = np.array(list(points_generator))

        # Check if point cloud is empty
        if points_arr.size == 0:
            return

        # Get total number of valid points
        num_points = points_arr.shape[0]

        # Calculate min and max for X, Y, Z
        min_xyz = np.min(points_arr, axis=0)
        max_xyz = np.max(points_arr, axis=0)

        # Print the exact expected output format
        print(f"Topic Name: {self.topic_name}")
        print(f"Timestamp: {timestamp:.3f}")
        print(f"Total number of points: {num_points}")
        print(f"X range: [{min_xyz[0]:.1f}, {max_xyz[0]:.1f}] m, "
              f"Y range: [{min_xyz[1]:.1f}, {max_xyz[1]:.1f}] m, "
              f"Z range: [{min_xyz[2]:.1f}, {max_xyz[2]:.1f}] m\n")

if __name__ == '__main__':
    rospy.init_node('pc_profiler_node', anonymous=True)

    TOPICS_TO_SUBSCRIBE = [
        # "/ouster/top_122219002200", # lidar
        "/ars548/radar_front/detections" # radar
    ]

    profilers = []
    for topic in TOPICS_TO_SUBSCRIBE:
        profilers.append(PointCloudProfiler(topic))

    rospy.spin()
