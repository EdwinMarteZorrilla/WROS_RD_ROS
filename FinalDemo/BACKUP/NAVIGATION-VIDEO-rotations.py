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
GRID_SIZE = 0.2          # meters per grid cell
OBSTACLE_DIST = 0.1      # 10 cm

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

# ---------------- LIDAR Reader ----------------
class LidarReader(Node):
    """Reads front obstacle distance from /scan_raw"""
    def __init__(self, topic='/scan_raw', stop_distance=OBSTACLE_DIST):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(LaserScan, topic, self.scan_callback, 10)
        self.front_distance = float('inf')
        self.stop_distance = stop_distance
        self.latest_scan = None

    def scan_callback(self, msg):
        """Compute min front range in ±15° window"""
        self.latest_scan = msg
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=float)
        valid = np.isfinite(ranges)
        angles, ranges = angles[valid], ranges[valid]

        window = np.deg2rad(15)
        mask = np.abs(angles) <= window
        front_ranges = ranges[mask]
        self.front_distance = np.min(front_ranges) if len(front_ranges) > 0 else float('inf')

    def obstacle_detected(self):
        return self.front_distance <= self.stop_distance

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
            angular_z = 1.2 * yaw_err
            angular_z = max(-0.6, min(0.6, angular_z))
            cos_yaw, sin_yaw = math.cos(fused_yaw), math.sin(fused_yaw)
            linear_x_body =  vx_map_unit * cos_yaw + vy_map_unit * sin_yaw
            linear_y_body = -vx_map_unit * sin_yaw + vy_map_unit * cos_yaw
            self.send_twist(linear_x_body, linear_y_body, angular_z, duration=0.05)
        self.stop()

    # ---------------- Robust Rotation Controller ----------------
    def rotate_to_yaw(self, target_yaw, odom_sub, yaw_tol=0.03, max_speed=1.2, timeout=4.0):
        Kp, Kd, Ki = 3.0, 0.8, 0.05
        integral, prev_err = 0.0, None
        prev_time = time.time()
        start_time = time.time()
        coarse_threshold = 0.25
        min_turn_speed = 0.20
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

        while rclpy.ok():
            if (time.time() - start_time) > timeout:
                self.get_logger().warn(f"rotate_to_yaw: timeout after {timeout}s")
                break
            rclpy.spin_once(odom_sub)
            now = time.time()
            dt = max(1e-4, now - prev_time)
            prev_time = now
            fused = odom_sub.fused_yaw
            yaw_err = math.atan2(math.sin(target_yaw - fused), math.cos(target_yaw - fused))
            abs_err = abs(yaw_err)
            if abs_err <= yaw_tol: break

            # coarse rotation
            if abs_err > coarse_threshold:
                ang_cmd = math.copysign(max_speed, yaw_err)
                twist = Twist()
                twist.angular.z = ang_cmd
                self.publisher_.publish(twist)
                time.sleep(0.01)
                prev_err = yaw_err
                continue

            # fine PD rotation
            derivative = (yaw_err - prev_err)/dt if prev_err is not None else 0.0
            prev_err = yaw_err
            integral += yaw_err*dt
            integral = max(-0.5, min(0.5, integral))
            ang_cmd = Kp*yaw_err + Ki*integral + Kd*derivative
            ang_cmd = max(-max_speed, min(max_speed, ang_cmd))
            if abs(ang_cmd) < min_turn_speed:
                ang_cmd = math.copysign(min_turn_speed, yaw_err)
            if abs_err < 0.12:
                scale = abs_err / 0.12
                ang_cmd *= scale
                if abs(ang_cmd) < 0.06:
                    ang_cmd = math.copysign(0.06, yaw_err)
            twist = Twist()
            twist.angular.z = ang_cmd
            self.publisher_.publish(twist)
            time.sleep(0.01)
        self.stop(duration=0.05)

# ---------------- Map Parsing and BFS ----------------
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
            if (0<=neighbor[0]<maze.shape[0] and 0<=neighbor[1]<maze.shape[1]
                and maze[neighbor]==0 and visited[neighbor]==0):
                frontier.append(neighbor)
                visited[neighbor] = 1
                parent[neighbor] = current
    path, node = [], goal
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
    plt.plot(start[1], start[0], "go", markersize=10, label="Start (R)")
    plt.plot(goal[1], goal[0], "yx", markersize=10, label="Goal (G)")
    plt.legend()
    plt.draw()
    plt.pause(0.001)

# ---------------- Follow Path with LIDAR ----------------
def follow_path(node, path, odom_sub, imu_sub, maze, start, goal, lidar_sub):
    plt.ion()
    i = 1
    prev_dr, prev_dc = 0, 0

    while i < len(path):
        cur = path[i-1]
        nxt = path[i]
        dr, dc = nxt[0]-cur[0], nxt[1]-cur[1]

        # group same-direction segments
        run_len = 1
        while (i + run_len < len(path) and
               path[i+run_len][0]-path[i+run_len-1][0]==dr and
               path[i+run_len][1]-path[i+run_len-1][1]==dc):
            run_len +=1
        total_distance = GRID_SIZE * run_len

        rclpy.spin_once(imu_sub)
        odom_sub.imu_yaw = imu_sub.yaw
        rclpy.spin_once(odom_sub)

        # ----- Rotation -----
        if (dr,dc)!=(prev_dr,prev_dc):
            if dr==-1 and dc==0: target_yaw=math.pi/2
            elif dr==1 and dc==0: target_yaw=-math.pi/2
            elif dr==0 and dc==1: target_yaw=0.0
            elif dr==0 and dc==-1: target_yaw=math.pi
            else: target_yaw=odom_sub.fused_yaw
            target_yaw=math.atan2(math.sin(target_yaw), math.cos(target_yaw))
            for _ in range(5):
                rclpy.spin_once(imu_sub); odom_sub.imu_yaw=imu_sub.yaw; rclpy.spin_once(odom_sub); time.sleep(0.02)
            node.rotate_to_yaw(target_yaw, odom_sub, yaw_tol=0.03, max_speed=1.2)
            prev_dr, prev_dc = dr, dc
            time.sleep(0.2)

        # ----- Forward Motion -----
        rclpy.spin_once(lidar_sub)
        if lidar_sub.obstacle_detected():
            print(f"⚠️ Obstacle at {lidar_sub.front_distance*100:.1f}cm — stopping")
            node.stop()
            # mark obstacle in maze
            next_cell = (round(-odom_sub.y_pos/GRID_SIZE)+dr, round(odom_sub.x_pos/GRID_SIZE)+dc)
            maze[next_cell] = 1
            cur_pos = (round(-odom_sub.y_pos/GRID_SIZE), round(odom_sub.x_pos/GRID_SIZE))
            path = bfs_path(maze, cur_pos, goal)
            i = 1
            continue

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
    lidar_reader = LidarReader()

    for _ in range(5):
        rclpy.spin_once(imu_reader)
        rclpy.spin_once(odom_reader)
        rclpy.spin_once(lidar_reader)
        time.sleep(0.05)

    odom_reader.imu_yaw = imu_reader.yaw
    rclpy.spin_once(odom_reader)

    maze_layout = [
        ["R","1","0","0","0","0"],
        ["0","1","0","1","1","0"],
        ["0","0","G","1","0","0"],
        ["0","1","1","1","0","1"],
        ["0","0","1","0","0","0"],
        ["0","1","1","1","1","0"]
    ]

    maze, start, goal = parse_map(maze_layout)
    path = bfs_path(maze, start, goal)
    plt.figure()
    plot_maze(maze, start, goal, path, robot_pos=(odom_reader.x_pos, odom_reader.y_pos))

    ORIENTATION_TO_YAW = {"north": math.pi/2,"south":-math.pi/2,"east":0.0,"west":math.pi}
    map_direction_to_align="south"
    target_map_yaw=ORIENTATION_TO_YAW[map_direction_to_align.lower()]

    # Compute map yaw offset
    if len(path)>=2:
        cur, nxt = path[0], path[1]
        dr, dc = nxt[0]-cur[0], nxt[1]-cur[1]
    else: dr, dc = 0,0

    current_fused = odom_reader.fused_yaw
    raw_offset = target_map_yaw - current_fused
    odom_reader.map_yaw_offset = math.atan2(math.sin(raw_offset), math.cos(raw_offset))
    print(f"Aligning: fused={current_fused:.3f}, target={target_map_yaw:.3f}, offset={odom_reader.map_yaw_offset:.3f}")

    if dr!=0 and dc==0:
        print("First move forward — skipping initial rotation")
    else:
        print("Rotating to map alignment...")
        node.rotate_to_yaw(target_map_yaw, odom_reader, yaw_tol=0.02, max_speed=1.2)
        print("Rotation complete.")

    follow_path(node, path, odom_reader, imu_reader, maze, start, goal, lidar_reader)

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    imu_reader.destroy_node()
    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
