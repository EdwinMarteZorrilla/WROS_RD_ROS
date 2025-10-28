#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import time
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import math

# ---------------- CONFIG ----------------
GRID_SIZE = 0.50  # meters per grid cell
LIDAR_TOPIC = '/scan_raw'
FRONT_WINDOW_DEG = 10            # front window
OBSTACLE_THRESHOLD_CM = 10.0
UPDATE_INTERVAL = 0.2

def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

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

# ---------------- CmdVel Publisher ----------------
class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def send_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=0.1):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(twist)
            time.sleep(0.05)

    def stop(self, duration=0.1):
        self.send_twist(0.0, 0.0, 0.0, duration)

    def move_direction(self, dr, dc, odom_sub, distance=GRID_SIZE, speed=0.4):
        rclpy.spin_once(odom_sub)
        start_x, start_y = odom_sub.x_pos, odom_sub.y_pos
        vx_map = dc * GRID_SIZE
        vy_map = -dr * GRID_SIZE
        mag = math.hypot(vx_map, vy_map)
        if mag == 0: return
        vx_map_unit = (vx_map / mag) * speed
        vy_map_unit = (vy_map / mag) * speed
        target_distance = distance
        desired_heading = odom_sub.fused_yaw
        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            dx = odom_sub.x_pos - start_x
            dy = odom_sub.y_pos - start_y
            traveled = math.hypot(dx, dy)
            if traveled >= target_distance:
                break
            fused_yaw = odom_sub.fused_yaw
            yaw_err = math.atan2(math.sin(desired_heading - fused_yaw),
                                 math.cos(desired_heading - fused_yaw))
            angular_z = max(-0.6, min(0.6, 1.2 * yaw_err))
            cos_yaw = math.cos(fused_yaw)
            sin_yaw = math.sin(fused_yaw)
            linear_x_body =  vx_map_unit * cos_yaw + vy_map_unit * sin_yaw
            linear_y_body = -vx_map_unit * sin_yaw + vy_map_unit * cos_yaw
            self.send_twist(linear_x=linear_x_body, linear_y=linear_y_body,
                            angular_z=angular_z, duration=0.05)
        self.stop()

    def rotate_to_yaw(self, target_yaw, odom_sub, yaw_tol=0.05, max_speed=0.4):
        while rclpy.ok():
            rclpy.spin_once(odom_sub)
            yaw_err = math.atan2(math.sin(target_yaw - odom_sub.fused_yaw),
                                 math.cos(target_yaw - odom_sub.fused_yaw))
            if abs(yaw_err) <= yaw_tol:
                break
            twist = Twist()
            twist.angular.z = max(-max_speed, min(max_speed, yaw_err))
            self.publisher_.publish(twist)
            time.sleep(0.05)
        self.stop()

# ---------------- Map Parsing & BFS ----------------
def parse_map(layout):
    maze = np.zeros((len(layout), len(layout[0])), dtype=int)
    start = goal = None
    for r, row in enumerate(layout):
        for c, val in enumerate(row):
            if val == "1": maze[r, c] = 1
            elif val == "R": start = (r, c); maze[r, c] = 0
            elif val == "G": goal = (r, c); maze[r, c] = 0
            else: maze[r, c] = 0
    if start is None or goal is None: raise ValueError("Map must contain 'R' and 'G'")
    return maze, start, goal

def bfs_path(maze, start, goal):
    visited = np.zeros_like(maze)
    parent = {}
    frontier = deque([start])
    visited[start] = 1
    directions = [(-1,0),(1,0),(0,-1),(0,1)]
    while frontier:
        current = frontier.popleft()
        if current == goal: break
        for d in directions:
            neighbor = (current[0]+d[0], current[1]+d[1])
            if (0 <= neighbor[0] < maze.shape[0] and 0 <= neighbor[1] < maze.shape[1] and
                maze[neighbor] == 0 and visited[neighbor] == 0):
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

# ---------------- Maze Plot ----------------
def plot_maze(maze, start, goal, path=None, robot_pos=None):
    plt.clf()
    plt.imshow(maze, cmap="gray_r")
    if path:
        px, py = zip(*path)
        plt.plot(py, px, "b.-", label="Path")
    if robot_pos:
        x, y = robot_pos
        gx = int(round(-y / GRID_SIZE))
        gy = int(round(x / GRID_SIZE))
        gx = np.clip(gx, 0, maze.shape[0]-1)
        gy = np.clip(gy, 0, maze.shape[1]-1)
        plt.plot(gy, gx, "ro", label="Robot")
    plt.plot(start[1], start[0], "go", markersize=10, label="Start")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal")
    plt.legend()
    plt.draw()
    plt.pause(0.001)

# ---------------- Follow Path ----------------
def follow_path(node, path, odom_sub, imu_sub, maze, start, goal):
    plt.ion()
    i = 1
    while i < len(path):
        cur = path[i-1]
        nxt = path[i]
        dr = nxt[0] - cur[0]
        dc = nxt[1] - cur[1]
        run_len = 1
        while (i + run_len < len(path) and
               path[i + run_len][0] - path[i + run_len - 1][0] == dr and
               path[i + run_len][1] - path[i + run_len - 1][1] == dc):
            run_len += 1
        total_distance = GRID_SIZE * run_len
        rclpy.spin_once(imu_sub)
        odom_sub.imu_yaw = imu_sub.yaw
        node.move_direction(dr, dc, odom_sub, distance=total_distance)
        i += run_len
        rclpy.spin_once(odom_sub)
        plot_maze(maze, start, goal, path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))

# ---------------- LIDAR Integration ----------------
class LidarFrontMonitor(Node):
    def __init__(self, odom_sub, maze, goal):
        super().__init__('lidar_monitor')
        self.odom_sub = odom_sub
        self.maze = maze
        self.goal = goal
        self.latest_scan = None
        self.path_update_callback = None
        self.subscription = self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, 10)
        self.timer = self.create_timer(UPDATE_INTERVAL, self.timer_callback)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def detect_front_obstacle(self):
        if self.latest_scan is None:
            return False
        angles = self.latest_scan.angle_min + np.arange(len(self.latest_scan.ranges)) * self.latest_scan.angle_increment
        ranges = np.array(self.latest_scan.ranges) * 100.0
        half_window = math.radians(FRONT_WINDOW_DEG/2)
        mask = np.abs(angles) <= half_window
        front_ranges = ranges[mask]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        if len(front_ranges) == 0: return False
        return float(np.min(front_ranges)) <= OBSTACLE_THRESHOLD_CM

    def timer_callback(self):
        if self.detect_front_obstacle():
            self.get_logger().warn("🚨 Obstacle <= 10cm detected in front, replanning path...")
            start_r = int(round(-self.odom_sub.y_pos / GRID_SIZE))
            start_c = int(round(self.odom_sub.x_pos / GRID_SIZE))
            start_r = np.clip(start_r, 0, self.maze.shape[0]-1)
            start_c = np.clip(start_c, 0, self.maze.shape[1]-1)
            new_path = bfs_path(self.maze, (start_r, start_c), self.goal)
            if self.path_update_callback:
                self.path_update_callback(new_path)

# ---------------- Main ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()
    imu_reader = IMUReader()

    # flush readings
    for _ in range(5):
        rclpy.spin_once(imu_reader)
        rclpy.spin_once(odom_reader)
        time.sleep(0.05)
    odom_reader.imu_yaw = imu_reader.yaw
    rclpy.spin_once(odom_reader)

    # Define maze
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

    plt.figure()
    plot_maze(maze, start, goal, path, robot_pos=(odom_reader.x_pos, odom_reader.y_pos))

    # --- LIDAR monitor ---
    lidar_monitor = LidarFrontMonitor(odom_reader, maze, goal)
    def update_path(new_path):
        nonlocal path
        path = new_path
    lidar_monitor.path_update_callback = update_path

    # --- Follow path ---
    follow_path(node, path, odom_reader, imu_reader, maze, start, goal)

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    imu_reader.destroy_node()
    lidar_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
