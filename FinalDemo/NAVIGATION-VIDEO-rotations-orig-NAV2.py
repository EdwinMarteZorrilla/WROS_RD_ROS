#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time
import numpy as np
import math
from collections import deque
import matplotlib.pyplot as plt

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2
ROBOT_INITIAL_HEADING_DEG = 180  # Robot starts facing South
FRONT_ANGLE_WINDOW_DEG = 15
OBSTACLE_DISTANCE_THRESHOLD = 0.10

# ---------------- IMU Reader ----------------
class IMUReader(Node):
    def __init__(self, topic='/imu'):
        super().__init__('imu_reader')
        self.yaw = 0.0
        self.subscription = self.create_subscription(Imu, topic, self.imu_callback, 10)

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

# ---------------- Odometry Reader ----------------
class OdometryReader(Node):
    def __init__(self, topic='/odom'):
        super().__init__('odom_reader')
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.odom_yaw = 0.0
        self.subscription = self.create_subscription(Odometry, topic, self.odom_callback, 10)

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

# ---------------- Nav2 Goal Sender ----------------
class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)
        self.get_logger().info(f"Goal sent: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°")

# ---------------- Front Lidar ----------------
class FrontLidar(Node):
    def __init__(self, topic='/scan'):
        super().__init__('front_lidar')
        self.min_distance = float('inf')
        self.subscription = self.create_subscription(LaserScan, topic, self.scan_callback, 10)

    def scan_callback(self, msg):
        total_points = len(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        front_angle_rad = math.radians(FRONT_ANGLE_WINDOW_DEG)

        start_idx = int((0 - front_angle_rad - angle_min) / angle_increment)
        end_idx = int((0 + front_angle_rad - angle_min) / angle_increment)
        start_idx = max(0, start_idx)
        end_idx = min(total_points - 1, end_idx)

        front_ranges = [r for r in msg.ranges[start_idx:end_idx+1] if r > 0.0]
        if front_ranges:
            self.min_distance = min(front_ranges)
        else:
            self.min_distance = float('inf')

    def is_obstacle_ahead(self):
        return self.min_distance < OBSTACLE_DISTANCE_THRESHOLD

# ---------------- BFS ----------------
def parse_map(layout):
    maze = np.zeros((len(layout), len(layout[0])), dtype=int)
    start = goal = None
    for r, row in enumerate(layout):
        for c, val in enumerate(row):
            if val == "1":
                maze[r, c] = 1
            elif val == "R":
                start = (r, c)
            elif val == "G":
                goal = (r, c)
    return maze, start, goal

def bfs_path(maze, start, goal):
    visited = np.zeros_like(maze)
    parent = {}
    frontier = deque([start])
    visited[start] = 1
    directions = [(-1,0),(1,0),(0,-1),(0,1)]
    while frontier:
        current = frontier.popleft()
        if current == goal:
            break
        for d in directions:
            neighbor = (current[0]+d[0], current[1]+d[1])
            if (0 <= neighbor[0] < maze.shape[0] and 0 <= neighbor[1] < maze.shape[1]
                and maze[neighbor] == 0 and visited[neighbor] == 0):
                frontier.append(neighbor)
                visited[neighbor] = 1
                parent[neighbor] = current
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = parent.get(node, start)
    path.append(start)
    path.reverse()
    return path

# ---------------- Follow path with live plot ----------------
def follow_path_live(nav2_client, lidar, odom_reader, path, maze):
    plt.figure()
    plt.imshow(maze, cmap="gray_r")
    plt.plot([c[1] for c in path], [c[0] for c in path], "b.-")
    plt.title("Robot BFS Navigation")
    robot_marker, = plt.plot([], [], "ro", markersize=10)
    plt.show(block=False)

    i = 1
    while i < len(path):
        cur = path[i-1]
        nxt = path[i]
        dr = nxt[0] - cur[0]
        dc = nxt[1] - cur[1]

        x_map = nxt[1] * GRID_SIZE
        y_map = (maze.shape[0] - 1 - nxt[0]) * GRID_SIZE

        yaw = 0.0
        if dr == -1 and dc == 0: yaw = math.pi/2
        elif dr == 1 and dc == 0: yaw = -math.pi/2
        elif dr == 0 and dc == 1: yaw = 0.0
        elif dr == 0 and dc == -1: yaw = math.pi
        yaw += math.radians(ROBOT_INITIAL_HEADING_DEG)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        if lidar.is_obstacle_ahead():
            nav2_client.get_logger().info("Front obstacle detected. Pausing...")
            while lidar.is_obstacle_ahead():
                time.sleep(0.05)
            nav2_client.get_logger().info("Path clear. Resuming.")

        nav2_client.send_goal(x_map, y_map, yaw)
        time.sleep(0.2)

        # Update robot marker using odometry
        robot_marker.set_data(odom_reader.x_pos / GRID_SIZE, maze.shape[0]-1 - odom_reader.y_pos / GRID_SIZE)
        plt.pause(0.05)
        i += 1

# ---------------- Main ----------------
def main():
    rclpy.init()
    nav2_client = Nav2Client()
    lidar = FrontLidar()
    odom_reader = OdometryReader()

    maze_layout = [
        ["R","1","0","0","0","0"],
        ["0","1","0","1","1","0"],
        ["0","0","0","1","0","0"],
        ["0","1","1","1","0","1"],
        ["0","0","1","0","0","0"],
        ["G","1","1","1","1","0"]
    ]
    maze, start, goal = parse_map(maze_layout)
    path = bfs_path(maze, start, goal)

    follow_path_live(nav2_client, lidar, odom_reader, path, maze)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


# ✅ Features Added

# Live robot marker (red dot) updated from odometry.

# BFS path is still plotted (blue line).

# Robot pauses if front obstacle detected, then resumes.

# Map stays open while robot moves.

# Real-time visual feedback simulates RViz without launching it.