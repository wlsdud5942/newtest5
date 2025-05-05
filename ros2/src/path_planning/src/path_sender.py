#!/usr/bin/env python3
import sys
import os
import json
import math
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Int32

# locate package resources
full_path = os.path.join(os.getcwd(), "ros2/src/path_planning")
sys.path.append(os.path.join(full_path, "src"))

from waypoint_interpolator import interpolate_path
from area_checker import check_area

class PathSenderNode(Node):
    def __init__(self):
        super().__init__('path_sender')
        
        # Publisher for planned path
        self.path_pub_x = self.create_publisher(Float32MultiArray, 'path_x', 10)
        self.path_pub_y = self.create_publisher(Float32MultiArray, 'path_y', 10)

        # Subscribers
        self.create_subscription(Point, 'location', self.location_callback, 10)
        self.create_subscription(Point, '/matched_start_point', self.SCNN_callback, 10) #get waypoint 
        self.create_subscription(Int32, '/lane_state', self.state_callback, 10)
        self.create_subscription(Int32, '/lane_detection/lane_state', self.state_callback, 10)

        # Data containers
        self.current_pos = (0.0, 0.0)
        self.SCNN = (0,0)
        self.lane_state = 0
        self.current_path = []
        self.finished = True  # Or False, based on logic
        self.current_area = 3

        # Set the threshold distance to consider as "reached"
        self.threshold = 0.5
        self.lastArea = 2 # no last Area

        self.rest_throttle = 0
        self.rest_steering_angle = 0

        # Directory for per-area JSON waypoint files
        self.config_dir = os.path.join(full_path, 'config')
        self.corner_1_pass_count = 1
        self.previous_area = 'none'

        self.get_logger().info("Path Sender up & running test ver")
        
        # Set the desired publishing frequency (in Hz)
        self.timer = self.create_timer(0.5, self.loop)  # 0.5 seconds interval
        self.is_final_point_reached = True  # To track if the final waypoint has been reached

    def load_waypoints(self, filepath):
        return np.loadtxt(filepath, delimiter=',')

    def location_callback(self, msg: Point):
        self.current_pos = (msg.x, msg.y)

    def SCNN_callback(self, msg: Point):
        self.SCNN = (msg.x, msg.y)

    def state_callback(self, msg: Int32):
        self.lane_state = msg.data

    def state_callback(self, msg: Int32):
        self.lane_state = msg.data

    def finishCheck(self, final_point, location):
        distance_final_point = math.sqrt((location[0] - final_point[0])**2 + (location[1] - final_point[1])**2)
        finish = distance_final_point <= self.threshold
        
        if finish:
            self.get_logger().info(f"Arrived at the waypoint") 
        
        return finish

    def publish_path(self):
        x_list = self.current_path[:, 0].tolist()
        y_list = self.current_path[:, 1].tolist()

        msgx = Float32MultiArray()
        msgy = Float32MultiArray()
        #msgLD = Float32()

        msgx.data = x_list
        msgy.data = y_list
        #msgLD.data = self.LD

        self.path_pub_x.publish(msgx)
        self.path_pub_y.publish(msgy)

        final_point = [x_list[len(x_list)-1], y_list[len(x_list)-1]]
        self.finished = self.finishCheck(final_point, self.current_pos)
        self.get_logger().info(f"publishing waypoints")

    def loop(self):
        # finished start with true
        if self.finished:
            # Get current segment based on SCNN waypoint
            # get scnn data
            self.get_logger().info(f"finished: {self.finished}")
            
            self.get_logger().info(f"previous: {self.previous_area}")
            
            self.current_area = check_area( self.lastArea, self.SCNN, self.lane_state)
            self.get_logger().info(f"current area: {self.current_area}")
            self.previous_area = self.current_area
            
            if self.current_area == 0:
                waypoint = 'a'
            elif self.current_area == 1:
                waypoint = 'b'
            elif self.current_area == 2:
                waypoint = 'c'
            elif self.current_area == 3:
                waypoint = 'd'
            elif self.current_area == 4:
                waypoint = 'e'
            elif self.current_area == 5:
                waypoint = 'f'
            elif self.current_area == 6:
                waypoint = 'g'
            elif self.current_area == 7:
                waypoint = 'h'
            elif self.current_area == 8:
                waypoint = 'i'
            elif self.current_area == 9:
                if self.corner_1_pass_count == 1:
                    waypoint = 'j'
                    self.corner_1_pass_count += 1
                    self.get_logger().info("case 9: first")
                elif self.corner_1_pass_count == 2:
                    waypoint = 'm'
                    self.corner_1_pass_count = 1
                    self.get_logger().info("case 9: second")
            elif self.current_area == 10:
                waypoint = 'k'
            elif self.current_area == 11:
                waypoint = 'l'
            else:
                self.get_logger().info("section not detected")
                self.finished = True
                return  # Make sure to exit early if no valid waypoint

            try:
                file_path = os.path.join(self.config_dir, f"{waypoint}.csv")
                self.current_path = self.load_waypoints(file_path)
                self.finished = False
                self.get_logger().info("waypoints loaded")

            except Exception as e:
                self.get_logger().warn(f"Failed to load waypoint, path: {file_path}: {e}")
                return   

        self.publish_path()

def main():
    rclpy.init()
    node = PathSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PathSenderNode...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
