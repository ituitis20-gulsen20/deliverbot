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
		self.subscription_next_point = self.create_subscription(PoseArray, '/next_point', self.next_point_callback, 10)
		self.route_publisher = self.create_publisher(PoseArray, '/route', 10)
		self.map = None
		self.width = 0
		self.height = 0
		self.resolution = 0.0
		self.origin_x = 0.0
		self.origin_y = 0.0
		self.is_next_point_running = False
		self.is_waypoint_running = False

	def map_callback(self, msg):
		self.width = msg.info.width
		self.height = msg.info.height
		self.resolution = msg.info.resolution
		self.origin_x = msg.info.origin.position.x
		self.origin_y = msg.info.origin.position.y
		self.map = [msg.data[i:i+self.width] for i in range(0, self.width * self.height, self.width)]
		self.map = self.generate_cost_map(self.map)

	def generate_cost_map(self, map):
		robot_radius = 0.15
		for i in range(len(map)):
			for j in range(len(map[i])):
				if map[i][j] >= 50:
					map[i][j] = 100
					continue
				for x_dist in range(0, int(robot_radius/self.resolution) +2):
					for y_dist in range(0, int(robot_radius/self.resolution) +2 - x_dist):
						if x_dist == 0 and y_dist == 0:
							continue
						if i+x_dist < self.height and j+y_dist<self.width and map[i + x_dist][j + y_dist] >= 50:
							map[i][j] = 100
							break
						if i+x_dist < self.height and j-y_dist>-1 and map[i + x_dist][j - y_dist] >= 50:
							map[i][j] = 100
							break
						if i-x_dist >-1 and j+y_dist<self.width and map[i - x_dist][j + y_dist] >= 50:
							map[i][j] = 100
							break
						if i-x_dist >-1 and j-y_dist>-1 and map[i - x_dist][j - y_dist] >= 50:
							map[i][j] = 100
							break
		return map

	def waypoint_callback(self, msg):
		if self.is_waypoint_running:
			return
		self.is_waypoint_running = True
		print("waypoint_callback")
		if self.map is None:
			return
		poses = msg.poses

		waypoints = self.generate_waypoints(poses[0].position.x, poses[0].position.y, poses[1].position.x, poses[1].position.y)
		route = self.waypoints_to_route(waypoints)
		self.route_publisher.publish(route)
		self.is_waypoint_running = False

	# first pose is current location, after that every consecutive two pose is source and destination
	def next_point_callback(self, msg):
		if self.is_next_point_running:
			return
		self.is_next_point_running = True
		print("next_point_callback")
		if self.map is None:
			return
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

		self.is_next_point_running = False

	def coord_to_index(self, x):
		return [int((x[0]-self.origin_x) / self.resolution), int((x[1]-self.origin_y) / self.resolution)]

	def generate_adj_matrix(self, robot_location, cargo_locations):
		adj_matrix = [[0 for i in range(len(cargo_locations) + 1)] for j in range(len(cargo_locations) + 1)]

		points = [self.coord_to_index(robot_location)] + [self.coord_to_index(cargo_location[0]) for cargo_location in cargo_locations]
		adj_matrix[0][:] = self.djikstra(0, points)

		for i in range(len(cargo_locations)):
			points = [j[0] for j in cargo_locations[:i]] + [cargo_locations[i][1]] + [j[0] for j in cargo_locations[i+1:]]
			adj_matrix[i+1][1:] = self.djikstra(i+1, points)
			adj_matrix[i+1][0] = math.inf

		return adj_matrix

	def djikstra(self, start_index, locations):
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
		x = int((x-self.origin_x) / self.resolution)
		y = int((y-self.origin_y) / self.resolution)
		x_goal = int((x_goal-self.origin_x) / self.resolution)
		y_goal = int((y_goal-self.origin_y) / self.resolution)
		waypoints = []
		waypoints.append([x, y])
		path = self.a_star([x, y], [x_goal, y_goal])
		for i in range(len(path)):
			if self.check_can_see(waypoints[-1][0], waypoints[-1][1], path[i][0], path[i][1]):
				continue
			waypoints.append(path[i-1])
		waypoints.append([x_goal, y_goal])
		print(waypoints)
		print(x, y)
		print(x_goal, y_goal)
		return waypoints

	def check_can_see(self, x, y, x_goal, y_goal):
		x1, y1 = x, y
		x2, y2 = x_goal, y_goal

		dx = abs(x2 - x1)
		dy = abs(y2 - y1)

		if x1 < x2:
			sx = 1
		else:
			sx = -1

		if y1 < y2:
			sy = 1
		else:
			sy = -1

		err = dx - dy

		while True:
			if map[y1][x1] >= 50:
				return False

			if x1 == x2 and y1 == y2:
				break

			e2 = 2 * err

			if e2 > -dy:
				err = err - dy
				x1 = x1 + sx

			if e2 < dx:
				err = err + dx
				y1 = y1 + sy

		return True

	def a_star(self, start, goal):
		visited = [[False for i in range(self.width)] for j in range(self.height)]
		came_from = [[None for i in range(self.width)] for j in range(self.height)]
		cost_so_far = [[math.inf for i in range(self.width)] for j in range(self.height)]
		cost_so_far[start[0]][start[1]] = 0
		queue = [(0.0, start)]
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
					new_cost = cost_so_far[current_location[0]][current_location[1]] + math.sqrt((neighbor[0] - current_location[0]) ** 2 + (neighbor[1] - current_location[1]) ** 2)
					if new_cost < cost_so_far[neighbor[0]][neighbor[1]]:
						cost_so_far[neighbor[0]][neighbor[1]] = new_cost
						heapq.heappush(queue, (new_cost + math.sqrt((neighbor[0] - goal[0]) ** 2 + (neighbor[1] - goal[1]) ** 2), neighbor))
						came_from[neighbor[0]][neighbor[1]] = current_location

		path = []
		current_location = goal
		while current_location != start:
			path.append(current_location)
			current_location = came_from[current_location[0]][current_location[1]]
		path.reverse()
		return path


	def points_to_posearray(self, waypoints):
		route = PoseArray()
		for i in range(len(waypoints)):
			pose = Pose()
			pose.position.x = float(waypoints[i][0]) * self.resolution + self.origin_x
			pose.position.y = float(waypoints[i][1]) * self.resolution + self.origin_y
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
