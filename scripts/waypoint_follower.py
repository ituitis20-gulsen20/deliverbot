#!/usr/bin/env python3
# -*- coding: utf-8 -*-
## EB
## waypoint_follower.py
## 
## YZV406E Assignment 2 Skeleton
## 
## Notes to consier: Few helper functions and code snippets are already given to you. Examine the code carefully beforehand.
##
## If you want to make use of the map, use occupancy_grid variable.
##
## 
## STUDENT_ID:<INSERT_HERE_WITHIN_ARROWS>
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, Pose
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
"""
HELPER FUNCTIONS
"""
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        


class Navigator(Node):
    """
    Navigator node to make robot go from location A to B. 
    [IMPORTANT]
    IMPLEMENT YOUR CODES WITHIN THIS CLASS (You can define helper functions outside the class if you want)
    [IMPORTANT]
    """
    def __init__(self):
        super().__init__('waypoint_follower')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.sub_route1 = self.create_subscription(
            PoseArray,
            '/route1',
            self.set_route1,
            10,
        )# subscribe first route
        self.sub_route2 = self.create_subscription(
            PoseArray,
            '/route2',
            self.set_route2,
            10
        ) # subscribe second route
        self.subscription_waypoint = self.create_subscription(
            PoseStamped,
            '/waypoint',
            self.waypoint_callback,
            10) # subscribe next waypoint
        self.publish_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.cli = self.create_client(GetMap, '/map_server/map')
        #/map_server/map
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()
        
        ### MAP ###
        self.occupancy_grid = [] # OCCUPANCY GRID MESSAGE VARIABLE. You can also use this knowledge
                                # to handle obstacle.
        self.map_origin = Pose() # origin as a Pose type.
        self.map_width = 0 # width
        self.map_height = 0 # height
        self.map_resolution = 0 # size of each grid cell
        ###     ###
        self.tf_buffer = Buffer() # for transformation
        self.tf_listener = TransformListener(self.tf_buffer, self) # for transformation
        self.goal_dist_thld = 0.2 # max acceptable distance between robot and the goal
        self.angle_err_thld = 0.5-0.5*math.cos(math.pi/4) # max acceptable angle error between robot and the goal
        self.is_route1_set= 0 # 0 or 1, if 1 route1 is acquired by the topic
        self.is_route2_set = 0 # 0 or 1, if 1 route2 is acquired by the topic
        self.route1 = PoseArray() # route1
        self.route2 = PoseArray() # route2
        self.waypoint = PoseStamped() # next waypoint
        self.prev_distance = 0
        self.chatty_map = False # you may set chatty_map to true if you want to see map on the terminal


        # Problem variables
        self.speed = 0.8
        self.scans = None
        self.angle_increment = None
        self.angle_err_thld = 0.4
        self.directions = {"left":math.inf,"right":math.inf,"front":math.inf,"behind":math.inf}

        self.new_point = [-1, -1]
        self.new_point_found = False
        self.new_point_reached = False


    def send_request(self):
        
        self.future = self.cli.call_async(self.req)

    def set_route1(self, msg):
        if (self.is_route1_set == 0):
            self.route1 = msg
            self.is_route1_set = 1
        else:
            pass

    def set_route2(self, msg):
        if (self.is_route2_set == 0):
            self.route2 = msg
            self.is_route2_set = 1
        else:
            pass

    def waypoint_callback(self,msg):
        self.waypoint = msg

    def scan_callback(self, msg):

        # Well Lets see which information can ve get from the laser scan data
        # Laser scan is an array of distances to obstacles!
        # self.get_logger().info('Number of points in laser scan is: '+ str(len(msg.ranges)),throttle_duration_sec=1)
        # self.get_logger().info('The distance to the front scanned point is: '+ str(msg.ranges[0]),throttle_duration_sec=1)
        # self.get_logger().info('The distance to the front scanned point is: '+ str(msg.ranges[-1]),throttle_duration_sec=1)
        # self.get_logger().info('The distance to the middle(behind) scanned point is: '+ str(msg.ranges[len(msg.ranges)//2]),throttle_duration_sec=1)
        # ## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
        # self.get_logger().info('The minimum angle scanned by the laser is: '+ str(msg.angle_min),throttle_duration_sec=1)
        # self.get_logger().info('The maximum angle scanned by the laser is: '+ str(msg.angle_max),throttle_duration_sec=1)
        # self.get_logger().info('The increment in the angles scanned by the laser is: '+ str(msg.angle_increment),throttle_duration_sec=1)
        # self.get_logger().info('The minimum range (distance) the laser can perceive is: '+ str(msg.range_min),throttle_duration_sec=1)
        # self.get_logger().info('The maximum range (distance) the laser can perceive is: '+ str(msg.range_max),throttle_duration_sec=1)
        
        scans = msg.ranges
        self.scans = scans
        self.angle_increment = msg.angle_increment

        # Get laser distances for specified directions (Some directions are optimised for better performance)
        self.directions["left"] = min(scans[len(scans)//4-7:len(scans)//4-5])
        self.directions["right"] = scans[-len(scans)//4]
        self.directions["front"] = scans[0]
        self.directions["behind"] = scans[len(scans)//2]

    def goal_dist(self, x, y, x_goal, y_goal):
        """
        Calculate the distance between a point and the goal position
        """
        return math.sqrt((x_goal-x)**2 + (y_goal-y)**2)
    
    def calculate_dist(self, x1, y1, x2, y2):
        """
        Calculate the distance between any two points
        """
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
    def calc_angle_to_goal(self, x, y):
        """
        Calculate the angle to the goal position
        """
        angle_to_goal = math.atan2(y-robotY_tf, x-robotX_tf)
        angle_to_goal -= robot_yaw
        
        # Correct angle by adding or subtracting 2pi to find closest rotation
        if abs(angle_to_goal + (2*math.pi)) < abs(angle_to_goal):
            angle_to_goal += 2*math.pi
        elif abs(angle_to_goal - (2*math.pi)) < abs(angle_to_goal):
            angle_to_goal -= 2*math.pi
    
        return angle_to_goal
    
    def search_new_point(self):
        scans = self.scans
        angle_increment = self.angle_increment

        # convert math.inf to 3.5 in scans
        for i in range(len(scans)):
            if scans[i] == math.inf:
                scans[i] = 3.5
        # Get shortest laser with distance greater than threshold but closest to front laser in angle
        
        selected_laser = 0
        for i in range(len(scans)):
            if scans[-i] > 3.0:
                selected_laser = -i
                selected_laser -= 15
                break
        
        # Get angle of selected laser
        angle = robot_yaw + angle_increment * selected_laser
        distance = 3.0

        # Get location of selected laser
        x = robotX_tf + distance * math.cos(angle)
        y = robotY_tf + distance * math.sin(angle)

        self.new_point = [x, y]

        self.get_logger().info(f'Selected laser: {str(selected_laser)}',throttle_duration_sec=0.5)
        # new point
        self.get_logger().info(f'New point: {str(self.new_point)}',throttle_duration_sec=0.5)

    
    def go_to_goal(self, x, y):
        """
        Rotate and move the robot to the goal position
        """
        twist_msg = Twist()

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(y-robotY_tf, x-robotX_tf)
        angle_to_goal -= robot_yaw

        if abs(angle_to_goal + (2*math.pi)) < abs(angle_to_goal):
            angle_to_goal += 2*math.pi
        elif abs(angle_to_goal - (2*math.pi)) < abs(angle_to_goal):
            angle_to_goal -= 2*math.pi

        self.get_logger().info(f'Angle to goal: {str(angle_to_goal)}',throttle_duration_sec=0.5)
        self.get_logger().info(f'speed scale: {str(abs(angle_to_goal/math.pi))}',throttle_duration_sec=0.5)

        twist_msg.linear.x = self.speed - self.speed * abs(angle_to_goal/(math.pi-0.4))
        twist_msg.angular.z = angle_to_goal * 1.6

        # Slow down if there is an obstacle in front
        if self.directions["front"] < 0.9:
            twist_msg.linear.x *= 0.4

        # Slow down further if robot is going to crash
        if self.directions["front"] < 0.4:
            twist_msg.linear.x *= 0.4

        self.publish_twist.publish(twist_msg)

    def odom_callback(self, msg):
        global robotX # global keyword makes the variable accessable even outside the function!
        global robotY # global keyword makes the variable accessable even outside the function!
        global robotX_tf # global keyword makes the variable accessable even outside the function!
        global robotY_tf # global keyword makes the variable accessable even outside the function!
        global robot_yaw # global keyword makes the variable accessable even outside the function!
        
        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y
        
        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
        try:
            # grab the latest available transform from the odometry frame 
            # (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        _,_,robot_orient_z = euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
        robotX_tf = t.transform.translation.x
        robotY_tf = t.transform.translation.y
        robot_yaw = robot_orient_z # # only need the z axis, degree of orientation, between pi and -pi
        #self.get_logger().info('X:'+str(robotX_tf),throttle_duration_sec=0.5) # once at a half of a second
        #self.get_logger().info('Y:'+str(robotY_tf),throttle_duration_sec=0.5) # once at a half of a second
        #self.get_logger().info('Yaw:'+str(robot_yaw),throttle_duration_sec=0.5) # once at a half of a second

        # TURTLEBOT: I HAVE NO CLUE, WHICH INFORMATION SHOULD I DEPEND FOR INTELLIGENT WAYPOINT FOLLOWING? HELP ME MY FELLOW ENGINEER! 
        # TURTLEBOT: I HAVE NO CLUE, WHICH INFORMATION SHOULD I DEPEND FOR INTELLIGENT WAYPOINT FOLLOWING? HELP ME MY FELLOW ENGINEER! 
        # Twist is a type of ROS Message that enables us to send velocity commands to the robot

        self.get_logger().info(f"waypoint x: {self.waypoint.pose.position.x} y: {self.waypoint.pose.position.y}",throttle_duration_sec=0.5)
        self.get_logger().info(f"robot x: {robotX_tf} y: {robotY_tf} yaw: {robot_yaw}",throttle_duration_sec=0.5)

        if self.directions["front"] < 0.4 and not self.new_point_found:
            self.search_new_point()
            self.new_point_found = True
        
        if self.new_point_found:
            self.get_logger().info(f"new point x: {self.new_point[0]} y: {self.new_point[1]}",throttle_duration_sec=0.5)
            self.go_to_goal(self.new_point[0], self.new_point[1])
            if self.calculate_dist(robotX_tf, robotY_tf, self.new_point[0], self.new_point[1]) < 0.25:
                self.new_point_found = False
        else:
            self.go_to_goal(self.waypoint.pose.position.x, self.waypoint.pose.position.y)
        
        if (self.chatty_map):
            # you may set chatty_map to true if you want to see map on the terminal
            # map is only acquired for once and does not change since then.
            self.get_logger().info(str(self.occupancy_grid))
            self.get_logger().info("Length of the map array:" + str(len(self.occupancy_grid)))
            self.get_logger().info("Height:" + str(self.map_height) + " Width:"+ str(self.map_height))
            self.get_logger().info("Origin of the map (Cell 0,0):" + str(self.map_origin))
            self.get_logger().info("Resolution (Size of each grid cell):" + str(self.map_resolution))

            self.chatty_map = False # avoid repetitive printing.
        

        
def main(args=None):
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    rclpy.init(args=args)

    navigator_node = Navigator()
    navigator_node.send_request() # send request to map server
    get_response=False
    while rclpy.ok():
        rclpy.spin_once(navigator_node) 
        if (navigator_node.future.done() & (not get_response)):
            # if future job is accomplished (GetMap) and not accomplished before
            navigator_node.get_logger().info("map is acquired")
            try:
                response = navigator_node.future.result() # get map response
                get_response = True # raise the response flag
                navigator_node.occupancy_grid= response.map.data # get the occupancy grid array
                navigator_node.map_height= response.map.info.height # get the occupancy grid array
                navigator_node.map_width= response.map.info.width # get the occupancy grid array
                navigator_node.map_origin= response.map.info.origin # get the occupancy grid array
                navigator_node.map_resolution= response.map.info.resolution # get the occupancy grid array
                
            except Exception as e:
                navigator_node.get_logger().info(e) # raise an error if response could not be acquired.
                get_response = False # lower the flag


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()