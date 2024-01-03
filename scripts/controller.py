#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, Pose
from rclpy.node import Node
from std_msgs.msg import String
import math


class Controller(Node):
	def __init__(self):
		super().__init__('Controller')
		self.publish_twist = self.create_publisher(Twist, '/cmd_vel', 10)
		self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
		self.subscription_loc = self.create_subscription(Pose, '/loc', self.loc_callback, 10)
		self.subscription_route = self.create_subscription(PoseArray, '/route', self.route_callback, 10)
		self.publish_string = self.create_publisher(String, '/tracker', 10)

		self.robotX = 0
		self.robotY = 0
		self.robot_yaw = 0

		self.route = None
		self.distance_threshold = 0.5

		self.speed = 0.8
		self.angle_increment = None
		self.angle_err_thld = 0.4
		self.directions = {"left": math.inf, "right": math.inf, "front": math.inf, "behind": math.inf}
		self.goal = {"x": 0, "y": 0}

	def loc_callback(self, msg):
		self.robotX = msg.position.x
		self.robotY = msg.position.y
		self.robot_yaw = msg.orientation.z
		if self.route is None:
			self.goal["x"] = self.robotX
			self.goal["y"] = self.robotY
			self.request_new_route()
			return
		if self.route is not None and len(self.route) > 0 and self.goal_dist(
								self.robotX, self.robotY, self.route[0].position.x,
								self.route[0].position.y) < self.distance_threshold:
			self.route.pop(0)
			self.goal["x"] = self.route[0].position.x
			self.goal["y"] = self.route[0].position.y
			print(self.goal)
	def route_callback(self, msg):
		self.route = msg.poses
		self.goal["x"] = self.route[0].position.x
		self.goal["y"] = self.route[0].position.y
		print(self.goal)

	def scan_callback(self, msg):
		scans = msg.ranges
		self.angle_increment = msg.angle_increment

		self.directions["left"] = min(scans[len(scans) // 4 - 7:len(scans) // 4 - 5])
		self.directions["right"] = scans[-len(scans) // 4]
		self.directions["front"] = scans[0]
		self.directions["behind"] = scans[len(scans) // 2]

		degree = int(math.degrees(self.calc_angle_to_goal(self.goal["x"], self.goal["y"])))
		dist_to_goal = self.goal_dist(self.robotX, self.robotY, self.goal["x"], self.goal["y"])

		if dist_to_goal < scans[degree] or dist_to_goal == math.inf:
			self.go_to_goal(self.goal["x"], self.goal["y"])
		elif self.directions["front"] < 0.4:
			self.request_new_route()
			twist = Twist()
			twist.linear.x = 0.0
			twist.angular.z = 0.0
			self.publish_twist.publish(twist)
		else:
			self.go_to_goal(self.goal["x"], self.goal["y"])

	def request_new_route(self):
		msg = String()
		msg.data = "new_route"
		print("requesting new route")
		self.publish_string.publish(msg)

	def goal_dist(self, x, y, x_goal, y_goal):
		return math.sqrt((x_goal - x) ** 2 + (y_goal - y) ** 2)

	def calculate_dist(self, x1, y1, x2, y2):
		return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

	def calc_angle_to_goal(self, x, y):
		angle_to_goal = math.atan2(y - self.robotY, x - self.robotX)
		angle_to_goal -= self.robot_yaw

		if abs(angle_to_goal + (2 * math.pi)) < abs(angle_to_goal):
			angle_to_goal += 2 * math.pi
		elif abs(angle_to_goal - (2 * math.pi)) < abs(angle_to_goal):
			angle_to_goal -= 2 * math.pi

		return angle_to_goal

	def go_to_goal(self, x, y):
		if self.route is None:
			return
		twist_msg = Twist()

		angle_to_goal = self.calc_angle_to_goal(x, y)

		twist_msg.linear.x = self.speed - self.speed * abs(angle_to_goal / (math.pi - 0.4))
		twist_msg.angular.z = angle_to_goal * 0.8

		if self.directions["front"] < 0.9:
			twist_msg.linear.x *= 0.4

		if self.directions["front"] < 0.4:
			twist_msg.linear.x *= 0.4

		if self.directions["front"] < 0.3:
			twist_msg.linear.x = 0.0

		self.publish_twist.publish(twist_msg)


def main(args=None):
	rclpy.init(args=args)

	controller_node = Controller()
	rclpy.spin(controller_node)

	controller_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
