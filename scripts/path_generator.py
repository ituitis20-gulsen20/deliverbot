#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from builtin_interfaces.msg import Time


class PathGenerator(Node):
	def __init__(self):
		self.subscription_waypoint = self.create_subscription(PoseArray, '/waypoint_generator', self.waypoint_callback, 10)
		self.subscription_next_point = self.create_subscription(Pose, '/next_point', self.next_point_callback, 10)
		self.route_publisher = self.create_publisher(PoseArray, '/route', 10)

	def waypoint_callback(self, msg):
		poses = msg.poses

		waypoints = self.generate_waypoints(poses[0].position.x, poses[0].position.y, poses[1].position.x, poses[1].position.y)
		route = self.waypoints_to_route(waypoints)
		self.route_publisher.publish(route)

	# first pose is current location, after that every consecutive two pose is source and destination
	def next_point_callback(self, msg):
		poses = msg.poses

		waypoints = self.generate_waypoints(poses[0].position.x, poses[0].position.y, poses[1].position.x,
											poses[1].position.y)
		route = self.waypoints_to_route(waypoints)
		self.route_publisher.publish(route)

	def generate_waypoints(self, x, y, x_goal, y_goal):
		waypoints = []
		waypoints.append([x, y])
		waypoints.append([x_goal, y_goal])
		return waypoints

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


def main(args=None):
	rclpy.init(args=args)
	path_generator = PathGenerator()
	rclpy.spin(path_generator)
	path_generator.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()