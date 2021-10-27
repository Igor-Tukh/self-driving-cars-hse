#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

eps = 1e-9
scale = 0.1

class Filter:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        self.filtered_points_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=2)
        self.rate = rospy.Rate(10)
		
    def laser_callback(self, msg):
        n = len(msg.ranges)
        angles = np.array([msg.angle_min + msg.angle_increment * i for i in range(n)])
        
        # I use the filtering method suggested during lecture:
        ranges = np.array(msg.ranges)
        diff = np.abs(ranges[2:] + ranges[:-2] - 2 * ranges[1:-1]) / 2
        # idea is to select the treshold dynamicly (another possible option is clusterization)
        # however, every dynamic threshold I tried looks so-so 
        threshold = np.median(diff) * 3
        # print(np.median(diff), diff.mean(), np.percentile(diff, 5), np.percentile(diff, 95), flush=True)
        original_mask = np.concatenate([[False], diff < threshold, [False]])
        mask = original_mask.copy()
        mask[:-1] |= (original_mask[1:] & (np.abs(ranges[1:] - ranges[:-1] < threshold)))
        mask[1:] |= (original_mask[:-1] & (np.abs(ranges[1:] - ranges[:-1] < threshold)))
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = 8  # POINTS
        marker.action = 0
        marker.id = 0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [Point(ranges[i] * np.cos(angles[i]), ranges[i] * np.sin(angles[i]), 0.) for i in range(n) if mask[i]]
        self.rate.sleep()
        self.filtered_points_publisher.publish(marker)
        
rospy.init_node('filter')
Filter()
rospy.spin()

