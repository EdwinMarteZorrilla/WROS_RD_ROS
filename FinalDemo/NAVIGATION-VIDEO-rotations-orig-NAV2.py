#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time
import numpy as np
import math
from collections import deque
import matplotlib.pyplot as plt

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2  # meters per grid cell
rot_speed = 0.8  # default rotation speed

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
        self.imu_yaw = 0.0
        self.map_yaw_offset = 0.0
        self.fused_yaw = 0.0
        self.subscription = self.create_subscription(Odometry, topic, self.odom_callback, 10)

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        alpha = 0.98
        geographic_yaw = alpha * self.odom_yaw + (1 - alpha) * self.imu_yaw
        self.fused_yaw = math.atan2(math.sin(geographic_yaw + self.map_yaw_offset),
                                    math.cos(geographic_yaw + self.map_yaw_offset))

# ---------------- Nav2 Goal Sender ----------------
class Nav2Client(Node):
    """Sends goals to Nav2 for autonomous movement and obstacle avoidance"""
    def __init__(self):
        super().__init__('nav2_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # Convert yaw to quaternion
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)

# ---------------- Map Parsing and BFS ----------------
def parse_map(layout):
    maze = np.zeros((len(layout), len(layout[0])), dtype=int)
    start = goal = None
    for r, row in enumerate(layout):
        for c, val in enumerate(row):
            if val == "1":
                maze[r, c] = 1
            elif val == "R":
                start = (r, c)
                maze[r, c] = 0
            elif val == "G":
                goal = (r, c)
                maze[r, c] = 0
            else:
                maze[r, c] = 0
    if start is None or goal is None:
        raise ValueError("Map must contain both 'R' (start) and 'G' (goal)!")
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

# ---------------- Follow Path Using Nav2 ----------------
def follow_path_nav2(nav2_client, path):
    """Send each grid cell as a goal to Nav2"""
    for i in range(1, len(path)):
        cur = path[i-1]
        nxt = path[i]
        dr = nxt[0] - cur[0]
        dc = nxt[1] - cur[1]

        # Compute map coordinates for the next cell
        x_map = nxt[1] * GRID_SIZE
        y_map = -nxt[0] * GRID_SIZE
        yaw = 0.0
        if dr == -1 and dc == 0:
            yaw = math.pi/2
        elif dr == 1 and dc == 0:
            yaw = -math.pi/2
        elif dr == 0 and dc == 1:
            yaw = 0.0
        elif dr == 0 and dc == -1:
            yaw = math.pi

        nav2_client.send_goal(x_map, y_map, yaw)
        # Give Nav2 time to reach the goal
        time.sleep(1.0)  # wait a bit for Nav2 to start
        # In a real implementation, you would wait for goal completion feedback

# ---------------- Main ----------------
def main():
    rclpy.init()
    nav2_client = Nav2Client()

    maze_layout = [
        ["R", "1", "0", "0", "0", "0"],
        ["0", "1", "0", "1", "1", "0"],
        ["0", "0", "G", "1", "0", "0"],
        ["0", "1", "1", "1", "0", "1"],
        ["0", "0", "1", "0", "0", "0"],
        ["0", "1", "1", "1", "1", "0"]
    ]

    maze, start, goal = parse_map(maze_layout)
    path = bfs_path(maze, start, goal)

    plt.figure()
    plt.imshow(maze, cmap="gray_r")
    plt.plot([c[1] for c in path], [c[0] for c in path], "b.-")
    plt.plot(start[1], start[0], "go")
    plt.plot(goal[1], goal[0], "yx")
    plt.pause(0.001)

    follow_path_nav2(nav2_client, path)

    rclpy.shutdown()

if __name__ == "__main__":
    main()