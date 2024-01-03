#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time
import math
import itertools
import heapq

class PathGenerator(Node):
	def __init__(self):
		super().__init__('PathGenerator')
		self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
		self.subscription_waypoint = self.create_subscription(PoseArray, '/waypoint_generator', self.waypoint_callback, 10)
		self.subscription_next_point = self.create_subscription(Pose, '/next_point', self.next_point_callback, 10)
		self.route_publisher = self.create_publisher(PoseArray, '/route', 10)
		self.map = []
		self.width = 0
		self.height = 0

	def map_callback(self, msg):
		self.map = msg.data
		self.width = msg.info.width
		self.height = msg.info.height

	def waypoint_callback(self, msg):
		poses = msg.poses

		waypoints = self.generate_waypoints(poses[0].position.x, poses[0].position.y, poses[1].position.x, poses[1].position.y)
		route = self.waypoints_to_route(waypoints)
		self.route_publisher.publish(route)

	# first pose is current location, after that every consecutive two pose is source and destination
	def next_point_callback(self, msg):
		poses = msg.poses

		next_point = 0

		cargo_locations = []

		for i in range(1, len(poses), 2):
			cargo_location = []
			cargo_location.append([poses[i].position.x, poses[i].position.y])
			cargo_location.append([poses[i+1].position.x, poses[i+1].position.y])
			cargo_locations.append(cargo_location)

		adj_matrix = self.generate_adj_matrix([poses[0].position.x, poses[0].position.y], cargo_locations)

		shortest_path = []
		shortest_path_length = math.inf

		for i in itertools.permutations(range(1, len(cargo_locations)+1)):
			path = [0] + list(i)
			path_length = 0
			for j in range(len(path)-1):
				if adj_matrix[path[j]][path[j+1]] == math.inf:
					continue
				path_length += adj_matrix[path[j]][path[j+1]]
			if path_length < shortest_path_length:
				shortest_path = path
				shortest_path_length = path_length

		next_point = shortest_path[1]

		waypoints = self.generate_waypoints(poses[0].position.x, poses[0].position.y, poses[2*next_point+1].position.x,
											poses[2*next_point+1].position.y)

		route = self.waypoints_to_route(waypoints)
		self.route_publisher.publish(route)

	def generate_adj_matrix(self, robot_location, cargo_locations):
		adj_matrix = [[0 for i in range(len(cargo_locations) + 1)] for j in range(len(cargo_locations) + 1)]

		points = [robot_location] + [cargo_location[0] for cargo_location in cargo_locations]
		adj_matrix[0][:] = self.djikstra(0, points)

		for i in range(len(cargo_locations)):
			points = [j[0] for j in cargo_locations[:i]] + [cargo_locations[i][1]] + [j[0] for j in cargo_locations[i+1:]]
			adj_matrix[i+1][1:] = self.djikstra(i+1, points)
			adj_matrix[i+1][0] = math.inf

		return adj_matrix

	def djikstra(self, start_index, locations: list[list[int, int]]):
		visited = [[False for i in range(self.width)] for j in range(self.height)]
		distances = [math.inf for i in range(len(locations))]
		distances[start_index] = 0
		queue = [(0.0, locations[start_index])]
		heapq.heapify(queue)

		while queue:
			current_distance, current_location = heapq.heappop(queue)
			if visited[current_location[0]][current_location[1]]:
				continue
			visited[current_location[0]][current_location[1]] = True
			for i in range(3):
				for j in range(3):
					if i == 1 and j == 1:
						continue
					neighbor = [current_location[0] + i - 1, current_location[1] + j - 1]
					if neighbor[0] < 0 or neighbor[0] >= self.height or neighbor[1] < 0 or neighbor[1] >= self.width:
						continue
					if self.map[neighbor[0] * self.width + neighbor[1]] >= 50:
						continue
					if visited[neighbor[0]][neighbor[1]]:
						continue
					heapq.heappush(queue, (current_distance + math.sqrt((neighbor[0] - current_location[0]) ** 2 + (neighbor[1] - current_location[1]) ** 2), neighbor))
		return distances

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
