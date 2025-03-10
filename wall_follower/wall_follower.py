
#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
import math

from wall_follower.visualization_tools import VisualizationTools

class WallFollower(Node):
   def __init__(self):
       super().__init__("wall_follower")
       # Declare parameters to make them available for use
       self.declare_parameter("scan_topic", "default")
       self.declare_parameter("drive_topic", "default")
       self.declare_parameter("side", rclpy.Parameter.Type.INTEGER)
       self.declare_parameter("velocity", rclpy.Parameter.Type.DOUBLE)
       self.declare_parameter("desired_distance", rclpy.Parameter.Type.DOUBLE)
       self.declare_parameter("lookahead_distance", rclpy.Parameter.Type.DOUBLE)

       # Fetch constants from the ROS parameter server
       # This is necessary for the tests to be able to test varying parameters!
       self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
       self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
       self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
       self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
       self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
       self.LOOKAHEAD_DISTANCE = self.get_parameter('lookahead_distance').get_parameter_value().double_value
       self.START_DISTANCE = -0.2
      
       self.add_on_set_parameters_callback(self.parameters_callback)
       
       # TODO: Initialize your publishers and subscribers here
       self.pub_drive = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
       self.pub_wall = self.create_publisher(Marker, 'wall_test', 10)
       self.get_logger().info("IN THE CONSTRUCTOR")
       
       self.sub_laser = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.new_scan,
            10)
       
       #self.prev_error = 0
       self.prev_dist = 0.0
       #self.kp = 0.4
       self.kp = 1.0
       self.kp_wall = 3

       self.kd = 0.0

   def new_scan(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        range_min = msg.range_min #  -2.3550000190734863
        range_max = msg.range_max #   2.3550000190734863
        ranges = msg.ranges
        angles = [angle_min + angle_increment * i for i in range(len(ranges)+1)]

        # Converts LIDAR readings into cartesian coordinates
        coordinates = [(ranges[i] * math.cos(angles[i]), ranges[i] * math.sin(angles[i])) for i in range(len(ranges))]

        # Front wall
        #front_distance = np.mean(ranges[530:550])
        if self.SIDE == 1:
            front_distance = np.mean(ranges[540:580])
        else:
            front_distance = np.mean(ranges[500:540])
        
        # Whether or not we're following left or right
        alpha_start = self.SIDE*(math.pi/2 - math.atan(self.START_DISTANCE/self.DESIRED_DISTANCE))
        alpha_end = self.SIDE*(math.pi/2 - math.atan(self.LOOKAHEAD_DISTANCE/self.DESIRED_DISTANCE))

        if self.SIDE == -1:
            i_start = int((alpha_start - angle_min)/angle_increment)
            i_end = int((alpha_end - angle_min)/angle_increment)
        else:
            i_end = int((alpha_start - angle_min)/angle_increment)
            i_start = int((alpha_end - angle_min)/angle_increment)

        coordinates = coordinates[i_start:i_end]

        # Split the coordinates for line fitting
        x = [c[0] for c in coordinates]
        y = [c[1] for c in coordinates]
        slope, intercept = np.polyfit(x, y, 1)

        # just for line plotting
        xs = [-2.5 + .1*i for i in range(50)]
        ys = [slope*i + intercept for i in xs]
        VisualizationTools.plot_line(xs, ys, self.pub_wall, frame="laser")
        #VisualizationTools.plot_line([0., 1.], [0., 1.], self.pub_wall, frame="laser")
        
        dist = abs(slope*(self.START_DISTANCE*3 + self.LOOKAHEAD_DISTANCE)/4 + intercept)

        if slope != 0:
            x_intercept = max(-intercept/slope, 0)
        else:
            x_intercept = float("inf")
        
        self.get_logger().info(f"Distance is: {dist}")
        error = (self.DESIRED_DISTANCE + 0.0) - dist

        error_wall = 0
        #if (x_intercept < 10) and (x_intercept > 0):
        wall_dist = 1.5
        #if x_intercept > 0:
        if front_distance < wall_dist:

            #error_wall = (1/x_intercept)
            #error_wall = (1/front_distance) ** 2
            error_wall = wall_dist - front_distance

            self.get_logger().info(f"{error_wall=}")
        
        drive_angle = -self.SIDE * (error * self.kp + self.kp_wall * error_wall + (dist - self.prev_dist)/(0.025) * self.kd)
        #drive_angle = -self.SIDE * (error * self.kp + (dist - self.prev_dist)/(0.025) * self.kd)
        speed = self.VELOCITY

        self.prev_dist = dist

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rclpy.time.Time().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(drive_angle)

        #drive_msg.drive.steering_angle = float(0)
        self.pub_drive.publish(drive_msg)

   def parameters_callback(self, params):
       """
       DO NOT MODIFY THIS CALLBACK FUNCTION!
      
       This is used by the test cases to modify the parameters during testing.
       It's called whenever a parameter is set via 'ros2 param set'.
       """
       for param in params:
           if param.name == 'side':
               self.SIDE = param.value
               self.get_logger().info(f"Updated side to {self.SIDE}")
           elif param.name == 'velocity':
               self.VELOCITY = param.value
               self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
           elif param.name == 'desired_distance':
               self.DESIRED_DISTANCE = param.value
               self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
       return SetParametersResult(successful=True)

def main():
   rclpy.init()
   wall_follower = WallFollower()
   rclpy.spin(wall_follower)
   wall_follower.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
  

