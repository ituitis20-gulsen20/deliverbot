#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
from std_msgs.msg import String
from builtin_interfaces.msg import Time

def euler_from_quaternion(x, y, z, w):
	t3 = 2.0 * (w * z + x * y)
	t4 = 1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)
	return yaw_z

class CargoTracker(Node):
	def __init__(self):
		super().__init__('cargo_tracker')

		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
	
		# self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
		self.subscription_controller = self.create_subscription(String, '/tracker', self.controller_message_callback, 10)
		self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
		self.pose_publisher = self.create_publisher(Pose, '/loc', 10)
		self.waypoint_publisher = self.create_publisher(PoseArray, '/waypoint_generator', 10)
		self.next_point_publisher = self.create_publisher(PoseArray, '/next_point', 10)

		# [[Source: [int, int], Destination: [int, int]]]
		self.cargo_locations = [
			[[5.5, 0.6], [5.5, 2.0]],
			[[4, 2.5], [1.7, 1.7]],
			[[3.6, -1.7], [2.3, -1.7]],
			[[0.3, -1.7], [0.3, 0.5]],
			[[2.0, 3.0], [3.0, 3.0]]
		]

		self.robotX_tf = 0
		self.robotY_tf = 0
		self.robot_yaw = 0

		self.carried_cargo = -1

		self.distance_threshold = 0.5

	def odom_callback(self, msg):
		robotX = msg.pose.pose.position.x
		robotY = msg.pose.pose.position.y

		to_frame_rel = "odom"
		from_frame_rel = "base_footprint"
		try:
			t = self.tf_buffer.lookup_transform(
				to_frame_rel,
				from_frame_rel,
				rclpy.time.Time())
		except TransformException as ex:
			self.get_logger().info(
				f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
			return
		self.robotX_tf = t.transform.translation.x
		self.robotY_tf = t.transform.translation.y
		self.robot_yaw = euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
													t.transform.rotation.w)
		pose = Pose()
		pose.position.x = self.robotX_tf
		pose.position.y = self.robotY_tf
		pose.orientation.z = self.robot_yaw

		self.pose_publisher.publish(pose)
		waypoints = None

		# self.get_logger().info(f'Robot x = {self.robotX_tf}, y = {self.robotY_tf}, yaw = {self.robot_yaw}')

		if self.carried_cargo == -1:
			for i in range(len(self.cargo_locations)):
				if self.goal_dist(robotX, robotY, self.cargo_locations[i][0][0], self.cargo_locations[i][0][1]) < self.distance_threshold:
					self.carried_cargo = i
					path = [[self.robotX_tf, self.robotY_tf], [self.cargo_locations[i][1][0], self.cargo_locations[i][1][1]]]
					route = self.points_to_posearray(path)
					self.waypoint_publisher.publish(route)

		else:
			if self.goal_dist(robotX, robotY, self.cargo_locations[self.carried_cargo][1][0],
								self.cargo_locations[self.carried_cargo][1][1]) < self.distance_threshold:
				self.carried_cargo = -1
				self.cargo_locations.pop(self.carried_cargo)

				cargo_locations = [[self.robotX_tf, self.robotY_tf]]
				for i in range(len(self.cargo_locations)):
					cargo_locations.append(self.cargo_locations[i][0])
					cargo_locations.append(self.cargo_locations[i][1])
				route = self.points_to_posearray(cargo_locations)
				self.next_point_publisher.publish(route)
				self.get_logger().info(f'Route {cargo_locations}')


	def points_to_posearray(self, waypoints):
		route = PoseArray()
		for i in range(len(waypoints)):
			pose = Pose()
			pose.position.x = float(waypoints[i][0])
			pose.position.y = float(waypoints[i][1])
			route.poses.append(pose)
		route.header.frame_id = "map"
		stamp = self.get_clock().now()
		route.header.stamp = Time()
		route.header.stamp.sec = stamp.nanoseconds // 10 ** 9
		route.header.stamp.nanosec = stamp.nanoseconds % 10 ** 9

		return route

	def goal_dist(self, x, y, x_goal, y_goal):
		return math.sqrt((x_goal - x) ** 2 + (y_goal - y) ** 2)

	def controller_message_callback(self, msg):
		if msg.data == "new_route":
			if self.carried_cargo != -1:
				path = [[self.robotX_tf, self.robotY_tf],
						[self.cargo_locations[self.carried_cargo][1][0], self.cargo_locations[self.carried_cargo][1][1]]]
				route = self.points_to_posearray(path)
				self.get_logger().info(f'Route {path}')
				self.waypoint_publisher.publish(route)
			else:
				cargo_locations = [[self.robotX_tf, self.robotY_tf]]
				for i in range(len(self.cargo_locations)):
					cargo_locations.append(self.cargo_locations[i][0])
					cargo_locations.append(self.cargo_locations[i][1])
				route = self.points_to_posearray(cargo_locations)
				self.next_point_publisher.publish(route)
				self.get_logger().info(f'Route {cargo_locations}')

def main(args=None):
	rclpy.init(args=args)

	cargo_tracker = CargoTracker()

	rclpy.spin(cargo_tracker)

	cargo_tracker.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()