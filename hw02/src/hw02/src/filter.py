#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

eps = 1e-9
scale = 0.1
map_size = 20
map_cells = 40

class Filter:
    def __init__(self):
        self.subscriber = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        self.filtered_points_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=2)
        self.map_publisher = rospy.Publisher('/map_topic', OccupancyGrid, queue_size=2)
        self.rate = rospy.Rate(10)
		
    def laser_callback(self, msg):
        n = len(msg.ranges)
        angles = np.array([msg.angle_min + msg.angle_increment * i for i in range(n)])
        # I use a variation of the filtering method suggested during lecture
        # General idea is to select the treshold dynamicly (another possible option is clusterization)
        # However, every dynamic threshold based approach I tried looks so-so :(
        ranges = np.array(msg.ranges)
        diff = np.abs(ranges[2:] + ranges[:-2] - 2 * ranges[1:-1]) / 2
        threshold = np.percentile(diff, 90)
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
        points = [(ranges[i] * np.cos(angles[i]), ranges[i] * np.sin(angles[i])) for i in range(n) if mask[i]]
        marker.points = [Point(*point, 0.) for point in points]
        self.rate.sleep()
        self.filtered_points_publisher.publish(marker)
        
        grid = OccupancyGrid()
        grid.header.frame_id = 'base_link'
        
        cell_size = 1. * map_size / map_cells
        
        grid.info.width = 2 * map_cells
        grid.info.height = 2 * map_cells
        grid.info.resolution = cell_size
        
        grid.info.origin.position.x = -map_size
        grid.info.origin.position.y = -map_size
        grid.info.origin.position.z = 0.0
        
        # Let's assume that only cells which contain points aren't free
        max_dist = None
        cells = np.ones((2 * map_cells, 2 * map_cells), dtype=np.int8)
        for x, y in points:
            dist = x**2 + y**2
            if max_dist is None or dist > max_dist:
                max_dist = dist
            cell_column = int(np.floor((x + map_size) / cell_size))
            cell_row = int(np.floor((y + map_size) / cell_size))
            if 0 <= cell_row < map_cells * 2 and 0 <= cell_column < map_cells * 2:
                cells[cell_row, cell_column] = 100
        # Let's also color the remote points as indefinite
        for r in range(2 * map_cells):
            for c in range(2 * map_cells):
                cx = grid.info.origin.position.x + cell_size * r + cell_size / 2
                cy = grid.info.origin.position.y + cell_size * c + cell_size / 2
                if cx**2 + cy**2 > max_dist:
                    cells[c, r] = 50
                
        grid.data = cells.reshape(-1)
        self.rate.sleep()
        self.map_publisher.publish(grid)
        
rospy.init_node('filter')
Filter()
rospy.spin()

