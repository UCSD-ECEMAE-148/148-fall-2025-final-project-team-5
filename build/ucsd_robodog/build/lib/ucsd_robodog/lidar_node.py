#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile

class LidarNode(Node):
	def __init__(self):
		super().__init__('lidar_node')
		self.subscriber_lidar = self.create_subscription(
			LaserScan, '/laser_scan', self.laserscan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
		self.get_logger().info("Lidar Node Ready...")

	def laserscan_callback(self, msg):
		ranges = msg.ranges
		len_ranges = len(ranges)
		self.get_logger().info(f'Length of ranges: {len_ranges}')

def main(args=None):
	rclpy.init(args=args)
	node = LidarNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
