  GNU nano 6.2                                                                                                                                                                                                                                                                                                                                                                                                                                                                     NAVEGATION                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import math
import sys #nuevo


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

# ---------------- Follow Path ----------------
def follow_path(node, path, odom_sub, imu_sub, maze, start, goal):
    plt.ion()
    i = 1
    while i < len(path):
        cur = path[i-1]
        nxt = path[i]
        dr = nxt[0] - cur[0]
        dc = nxt[1] - cur[1]

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

    # flush initial readings
    for _ in range(5):
        rclpy.spin_once(imu_reader)
        rclpy.spin_once(odom_reader)
        time.sleep(0.05)

    odom_reader.imu_yaw = imu_reader.yaw
    rclpy.spin_once(odom_reader)

  # ---------------- Automatic Goal Placement ----------------
    #Zone_ID = 1  # <-- Set this dynamically or manually
    # Get Zone_ID from command-line argument if provided
   # Get RPI_ID from command-line argument (e.g., "RPI_1" or "RPI_2")
    if len(sys.argv) > 1:
        rpi_id = sys.argv[1].strip().upper()
    else:
        rpi_id = "RPI_1"  # default

    # Map RPI_ID strings to Zone_ID numbers
    rpi_to_zone = {
        "RPI_1": 1,
        "RPI_2": 2
    }
    
    Zone_ID = rpi_to_zone.get(rpi_id, 1)  # default to Zone 1 if unknown
    print(f"[NAVIGATION] Received {rpi_id}, setting Zone_ID = {Zone_ID}")


    # Define XgoaXl coordinates based on Zone_ID
    zone_goals = {
        1: (4,0),  # (row, col)
        2: (2, 2)
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
        time.sleep(0.02)

    current_fused = odom_reader.fused_yaw
    raw_offset = target_map_yaw - current_fused
    odom_reader.map_yaw_offset = math.atan2(math.sin(raw_offset), math.cos(raw_offset))

    print(f"Aligning: current fused_yaw={current_fused:.3f}, target={target_map_yaw:.3f}, "
          f"offset={odom_reader.map_yaw_offset:.3f}")

    # -X-- Rotate physicalxly to face map forward ---
    print("Rotating robot to align forward with map direction...")
    node.rotate_to_yaw(target_map_yaw, odom_reader, yaw_tol=0.02, max_speed=0.3)
    print("Rotation complete. Robot is now facing map forward.")

    follow_path(node, path, odom_sub=odom_reader, imu_sub=imu_reader,
                maze=maze, start=start, goal=goal)

    node.stop()
    node.destroy_node()
    odom_reader.destroy_node()
    imu_reader.destroy_node()
    rclpy.shutdown()
   
if __name__ == "__main__":
    main()

