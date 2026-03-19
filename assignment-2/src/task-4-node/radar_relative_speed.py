#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import statistics
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

class RadarRelativeSpeedNode:
    def __init__(self):
        self.sub = rospy.Subscriber('/ars548/radar_front/detections', PointCloud2, self.pc_callback)
        self.pub = rospy.Publisher('/radar_speed_markers', MarkerArray, queue_size=10)

        self.tracked_objects = {}  # {id: (x, y)}
        self.next_object_id = 0
        self.max_track_dist = 5.0 # m

    def pc_callback(self, msg):
        marker_array = MarkerArray()

        # Remove previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Read points from PointCloud2
        try:
            points_gen = pc2.read_points(msg, field_names=('x', 'y', 'z', 'RangeRate'), skip_nans=True)
            all_points = list(points_gen)
        except Exception:
            return

        if not all_points:
            self.pub.publish(marker_array)
            return

        # Calculate background velocity
        range_rates = [p[3] for p in all_points]
        bg_velocity = statistics.median(range_rates)
        static_threshold = 2.0

        # Cluster points based on proximity and range rate difference from background velocity
        clusters = []
        for p in all_points:
            x, y, z, range_rate = p

            if math.hypot(x, y) > 100.0:
                continue

            if abs(range_rate - bg_velocity) < static_threshold:
                continue

            added = False
            for cluster in clusters:
                cx, cy = cluster[0][0], cluster[0][1]
                if math.hypot(x - cx, y - cy) < 3.0:
                    cluster.append((x, y, z, range_rate))
                    added = True
                    break

            if not added:
                clusters.append([(x, y, z, range_rate)])

        if not clusters:
            self.tracked_objects.clear()
            self.pub.publish(marker_array)
            return

        # Create Marker for points
        points_marker = Marker()
        points_marker.header = msg.header
        points_marker.ns = "colored_spheres"
        points_marker.id = 999
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD

        points_marker.scale.x = 0.2
        points_marker.scale.y = 0.2
        points_marker.scale.z = 0.2
        points_marker.color.a = 1.0

        # Track objects and assign IDs
        current_frame_objects = {}
        available_tracks = dict(self.tracked_objects)

        for i, cluster in enumerate(clusters):
            avg_x = sum(pt[0] for pt in cluster) / len(cluster)
            avg_y = sum(pt[1] for pt in cluster) / len(cluster)
            avg_z = sum(pt[2] for pt in cluster) / len(cluster)
            avg_range_rate = sum(pt[3] for pt in cluster) / len(cluster)
            display_speed = abs(avg_range_rate)

            # Find the closest historical track to this cluster
            best_id = -1
            min_dist = self.max_track_dist

            for track_id, (prev_x, prev_y) in available_tracks.items():
                dist = math.hypot(avg_x - prev_x, avg_y - prev_y)
                if dist < min_dist:
                    min_dist = dist
                    best_id = track_id

            if best_id != -1:
                current_frame_objects[best_id] = (avg_x, avg_y)
                del available_tracks[best_id] # Remove matched track from available tracks
            else:
                # Assign new ID
                best_id = self.next_object_id
                self.next_object_id += 1
                current_frame_objects[best_id] = (avg_x, avg_y)

            # Assign color
            hue = (best_id * 1.0 / 15.0) % 1.0
            r, g, b = self._hsv_to_rgb(hue, 1.0, 1.0)
            cluster_color = ColorRGBA(r=r, g=g, b=b, a=1.0)

            for pt in cluster:
                p = Point(x=pt[0], y=pt[1], z=pt[2])
                points_marker.points.append(p)
                points_marker.colors.append(cluster_color)

            # Create text marker
            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = "relative_speed_text"
            text_marker.id = best_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = avg_x
            text_marker.pose.position.y = avg_y
            text_marker.pose.position.z = avg_z + 2.5

            text_marker.scale.z = 1.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0

            text_marker.text = f"ID: {best_id}\n{display_speed:.1f} m/s"

            marker_array.markers.append(text_marker)

        self.tracked_objects = current_frame_objects

        if points_marker.points:
            marker_array.markers.append(points_marker)

        self.pub.publish(marker_array)

    def _hsv_to_rgb(self, h, s, v):
        if s == 0.0: return v, v, v
        i = int(h*6.0)
        f = (h*6.0) - i
        p, q, t = v*(1.0-s), v*(1.0-s*f), v*(1.0-s*(1.0-f))
        i %= 6
        if i == 0: return v, t, p
        if i == 1: return q, v, p
        if i == 2: return p, v, t
        if i == 3: return p, q, v
        if i == 4: return t, p, v
        if i == 5: return v, p, q

if __name__ == '__main__':
    rospy.init_node('radar_relative_speed_node', anonymous=False)
    node = RadarRelativeSpeedNode()
    rospy.spin()
