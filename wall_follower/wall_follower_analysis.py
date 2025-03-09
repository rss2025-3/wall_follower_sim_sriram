#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
from rclpy.time import Time
import numpy as np
import math

class WallFollowerAnalyzer(Node):
    def __init__(self):
        super().__init__("wall_follower_analyzer")

        self.declare_parameter("desired_distance", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("lookahead_distance", rclpy.Parameter.Type.DOUBLE)
        
        self.SCAN_TOPIC = "/scan"
        self.SIDE = -1  # -1 for right side, 1 for left side

        self.DESIRED_DISTANCE = 0.5
        self.LOOKAHEAD_DISTANCE = 2.0
        self.START_DISTANCE = -0.2
        
        self.sub_laser = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.analyze_scan,
            10)
        
        self.csv_file = open('wall_following_metrics.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'side_distance'])
        
        self.start_time = None

    def analyze_scan(self, msg):
        if self.start_time is None:
            self.start_time = Time.from_msg(msg.header.stamp)
        
        current_time = Time.from_msg(msg.header.stamp)
        time_since_start = (current_time - self.start_time).nanoseconds / 1e9
        
        # Calculate angles like in wall_follower.py
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        angles = [angle_min + angle_increment * i for i in range(len(ranges)+1)]

        alpha_start = self.SIDE*(math.pi/2 - math.atan(self.START_DISTANCE/self.DESIRED_DISTANCE))
        alpha_end = self.SIDE*(math.pi/2 - math.atan(self.LOOKAHEAD_DISTANCE/self.DESIRED_DISTANCE))

        if self.SIDE == -1:
            i_start = int((alpha_start - angle_min)/angle_increment)
            i_end = int((alpha_end - angle_min)/angle_increment)
        else:
            i_end = int((alpha_start - angle_min)/angle_increment)
            i_start = int((alpha_end - angle_min)/angle_increment)

        # Convert ranges to coordinates
        coordinates = [(ranges[i] * math.cos(angles[i]), ranges[i] * math.sin(angles[i])) 
                      for i in range(len(ranges))]
        coordinates = coordinates[i_start:i_end]
        
        # Fit line to points
        x = [c[0] for c in coordinates]
        y = [c[1] for c in coordinates]
        slope, intercept = np.polyfit(x, y, 1)
        
        #dist = abs(slope*(self.START_DISTANCE*3 + self.LOOKAHEAD_DISTANCE)/4 + intercept)
        dist = abs(intercept)
        
        self.get_logger().info(f"Distance is: {dist}")
        
        # Log the data
        self.csv_writer.writerow([time_since_start, dist])
        
    # def __del__(self):
    #     self.csv_file.close()

def main():
    rclpy.init()
    analyzer = WallFollowerAnalyzer()
    rclpy.spin(analyzer)
    analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()