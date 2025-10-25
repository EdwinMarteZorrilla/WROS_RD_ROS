#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan                # <-- added
import time
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import math
import heapq                                        # <-- for A*

# ---------------- CONFIG ----------------
GRID_SIZE = 0.2  # meters per grid cell

# ---------------- IMU Reader ----------------
class IMUReader(Node):
    """Reads IMU yaw from /imu"""
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
    """Reads robot position (x,y) and yaw from /odom, fused with IMU yaw"""
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

# ---------------- LIDAR Reader (NEW) ----------------
class LidarReader(Node):
    """Keeps latest LaserScan and offers a simple 'obstacle ahead' test"""
    def __init__(self, topic='/scan_raw'):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(LaserScan, topic, self.scan_callback, 10)
        self.latest_scan = None
        # tune as needed
        self.default_window_deg = 20

    def scan_callback(self, msg):
        self.latest_scan = msg

    def is_obstacle_ahead(self, threshold_m=0.20, window_deg=None):
        """
        Returns True if any valid range within +/- window_deg around 0 radians (robot front)
        is <= threshold_m. If no scan yet, returns False (be conservative? could instead True).
        """
        if self.latest_scan is None:
            return False
        if window_deg is None:
            window_deg = self.default_window_deg

        angles = self.latest_scan.angle_min + np.arange(len(self.latest_scan.ranges)) * self.latest_scan.angle_increment
        ranges = np.array(self.latest_scan.ranges, dtype=float)

        # mask finite ranges only
        finite_mask = np.isfinite(ranges)
        angles = angles[finite_mask]
        ranges = ranges[finite_mask]

        # look for angles near 0 (front). wrap-around safe
        diffs = np.abs(np.arctan2(np.sin(angles - 0.0), np.cos(angles - 0.0)))
        window_rad = math.radians(window_deg)
        mask = diffs <= window_rad
        if not np.any(mask):
            return False
        min_range = np.min(ranges[mask])
        return min_range <= threshold_m

# ---------------- Command Velocity Publisher ----------------
class CmdVelPublisher(Node):
    """Publishes Twist commands to control the Mecanum chassis"""
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

    # ---------------- Holonomic Motion ----------------
    def move_direction(self, dr, dc, odom_sub, distance=GRID_SIZE, speed=0.2):
        """
        Move the robot in the direction defined by grid deltas:
        dr = change in row (positive = down/south)
        dc = change in col (positive = right/east)
        """
        rclpy.spin_once(odom_sub)
        start_x, start_y = odom_sub.x_pos, odom_sub.y_pos

        # Map-frame vector in meters
        vx_map = dc * GRID_SIZE
        vy_map = -dr * GRID_SIZE

        mag = math.hypot(vx_map, vy_map)
        if mag == 0:
            return

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
            angular_z = 1.2 * yaw_err
            angular_z = max(-0.6, min(0.6, angular_z))

            cos_yaw = math.cos(fused_yaw)
            sin_yaw = math.sin(fused_yaw)
            linear_x_body =  vx_map_unit * cos_yaw + vy_map_unit * sin_yaw
            linear_y_body = -vx_map_unit * sin_yaw + vy_map_unit * cos_yaw

            print(f"[move_dir] fused_yaw={fused_yaw:+.2f}, yaw_err={yaw_err:+.2f}, "
                  f"vx_body={linear_x_body:+.2f}, vy_body={linear_y_body:+.2f}, ang_z={angular_z:+.2f}")

            self.send_twist(
                linear_x=linear_x_body,
                linear_y=linear_y_body,
                angular_z=angular_z,
                duration=0.05
            )
        self.stop()

    # ---------------- Rotate to a specific yaw ----------------
    def rotate_to_yaw(self, target_yaw, odom_sub, yaw_tol=0.05, max_speed=0.3):
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

# ---------------- A* Planner (NEW, lightweight) ----------------
def a_star(maze, start, goal):
    """
    Returns a path list of (r,c) from start to goal using A* with Manhattan heuristic.
    If no path, returns [].
    """
    rows, cols = maze.shape
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))
    came_from = {}
    gscore = {start: 0}
    visited = set()

    directions = [(-1,0),(1,0),(0,-1),(0,1)]
    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            # reconstruct
            path = []
            node = current
            while node != start:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path

        for d in directions:
            nb = (current[0]+d[0], current[1]+d[1])
            if not (0 <= nb[0] < rows and 0 <= nb[1] < cols):
                continue
            if maze[nb] == 1:
                continue
            tentative_g = gscore[current] + 1
            if tentative_g < gscore.get(nb, float('inf')):
                came_from[nb] = current
                gscore[nb] = tentative_g
                f = tentative_g + heuristic(nb, goal)
                heapq.heappush(open_set, (f, tentative_g, nb))
    return []  # no path

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
    plt.plot(start[1], start[0], "go", markersize=10, label="Start (R)")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal (G)")
    plt.legend()
    plt.draw()
    plt.pause(0.001)

# ---------------- Helpers ----------------
def world_to_grid(x, y, maze):
    """
    Convert robot world x,y (meters) into (row, col) indices used by maze.
    Uses the same mapping as plot_maze:
      row = round(-y / GRID_SIZE)
      col = round(x / GRID_SIZE)
    and clips to maze bounds.
    """
    r = int(round(-y / GRID_SIZE))
    c = int(round(x / GRID_SIZE))
    r = np.clip(r, 0, maze.shape[0]-1)
    c = np.clip(c, 0, maze.shape[1]-1)
    return (r, c)

# ---------------- Follow Path (modified) ----------------
def follow_path(node, path, odom_sub, imu_sub, maze, start, goal, lidar_sub=None):
    plt.ion()
    i = 1
    while i < len(path):
        cur = path[i-1]
        nxt = path[i]
        dr = nxt[0] - cur[0]
        dc = nxt[1] - cur[1]

        # BEFORE moving, check LIDAR for obstacle in heading direction for the *next* grid
        # We only check forward cone (robot's front); if obstacle detected, mark 'nxt' blocked and replan.
        if lidar_sub is not None:
            # spin to update lidar + odom/imu
            rclpy.spin_once(lidar_sub)
            rclpy.spin_once(odom_sub)
            rclpy.spin_once(imu_sub)
            odom_sub.imu_yaw = imu_sub.yaw

            obstacle_close = lidar_sub.is_obstacle_ahead(threshold_m=0.20, window_deg=20)
            if obstacle_close:
                print(f"[LIDAR] Obstacle detected ahead when about to move from {cur} to {nxt}. Marking cell blocked and replanning.")
                maze[nxt] = 1  # mark the next grid as blocked
                # compute robot's current cell to use as new start
                rclpy.spin_once(odom_sub)
                cur_world = (odom_sub.x_pos, odom_sub.y_pos)
                current_cell = world_to_grid(cur_world[0], cur_world[1], maze)
                # replan using A*
                new_path = a_star(maze, current_cell, goal)
                if not new_path:
                    print("[REPLAN] No path found after marking obstacle. Stopping.")
                    node.stop()
                    return
                # replace path from current position
                print(f"[REPLAN] New path length: {len(new_path)}. Switching to new plan.")
                path = new_path
                # find index of next step (first step after current cell)
                # If current_cell equals start of new_path, we set i=1 to follow new_path from new_path[0]->new_path[1]
                i = 1
                start = current_cell
                plot_maze(maze, start, goal, path, robot_pos=(odom_sub.x_pos, odom_sub.y_pos))
                continue  # loop to evaluate the new path's next move

        # Combine consecutive moves in same direction
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

# ---------------- Main ----------------
def main():
    rclpy.init()
    node = CmdVelPublisher()
    odom_reader = OdometryReader()
    imu_reader = IMUReader()
    lidar_reader = LidarReader()   # <-- new

    # flush initial readings
    for _ in range(5):
        rclpy.spin_once(imu_reader)
        rclpy.spin_once(odom_reader)
        rclpy.spin_once(lidar_reader)
        time.sleep(0.05)

    odom_reader.imu_yaw = imu_reader.yaw
    rclpy.spin_once(odom_reader)

  # ---------------- Automatic Goal Placement ----------------
    Zone_ID = 1  # <-- Set this dynamically or manually

    # Define goal coordinates based on Zone_ID
    zone_goals = {
        1: (5, 5),  # (row, col)
        2: (3, 4)
    }

    goal_coord = zone_goals.get(Zone_ID, (5, 5))  # Default if Zone_ID not found
        
    maze_layout = [
        ["R", "1", "0", "0", "0", "0"],
        ["0", "1", "0", "1", "1", "0"],
        ["0", "0", "0", "1", "0", "0"],
        ["0", "1", "1", "1", "0", "1"],
        ["0", "0", "1", "0", "0", "0"],
        ["0", "1", "1", "1", "1", "0"]
    ]

    # Place the goal automatically
    gr, gc = goal_coord
    maze_layout[gr][gc] = "G"

    maze, start, goal = parse_map(maze_layout)
    path = bfs_path(maze, start, goal)

    plt.figure()
    plot_maze(maze, start, goal, path, robot_pos=(odom_reader.x_pos, odom_reader.y_pos))

    # --- Orientation alignment ---
    ORIENTATION_TO_YAW = {
        "north": math.pi/2,
        "south": -math.pi/2,
        "east": 0.0,
        "west": -math.pi
    }

    map_direction_to_align = "south"
    target_map_yaw = ORIENTATION_TO_YAW[map_direction_to_align.lower()]

    # wait & get stable yaw
    for _ in range(10):
        rclpy.spin_once(imu_reader)
        rclpy.spin_once(odom_reader)
        rclpy.spin_once(lidar_reader)
        time.sleep(0.02)

    current_fused = odom_reader.fused_yaw
    raw_offset = target_map_yaw - current_fused
    odom_reader.map_yaw_offset = math.atan2(math.sin(raw_offset), math.cos(raw_offset))

    print(f"Aligning: current fused_yaw={current_fused:.3f}, target={target_map_yaw:.3f}, "
          f"offset={odom_reader.map_yaw_offset:.3f}")

    # --- Rotate physically to face map forward ---
    print("Rotating robot to align forward with map direction...")
    node.rotate_to_yaw(target_map_yaw, odom_reader, yaw_tol=0.02, max_speed=0.3)
    print("Rotation complete. Robot is now facing map forward.")

    # Pass lidar_reader into follow_path so it can check before every grid-step
    follow_path(node, path, odom_sub=odom_reader, imu_sub=imu_reader,
                maze=maze, start=start, goal=goal, lidar_sub=lidar_reader)

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    imu_reader.destroy_node()
    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
