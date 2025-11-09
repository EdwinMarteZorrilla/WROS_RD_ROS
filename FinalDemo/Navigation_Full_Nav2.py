#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import math
import time

# ======================================================
# --- USER SETTINGS ---
# ======================================================

GRID_RESOLUTION = 0.2   # meters per cell
WALL_VALUE = 100
FREE_VALUE = 0
UNKNOWN_VALUE = -1

# Robot heading in the real world when placed physically
# 0°=East, 90°=North, 180°=West, -90°=South
ROBOT_HEADING_DEG =  0.0

# Goal orientation at destination
GOAL_HEADING_DEG = 0.0

# Define your grid layout ('R' = robot start, 'G' = goal)
GRID_LAYOUT = [
   # ["R","0","0","0","0","G"],
    #["0","0","0","0","0","0"],
   # ["0","0","0","0","0","0"],
    #["0","0","0","0","0","0"],
    #["0","0","0","0","0","0"]

   ["R","0","0","0","0","0","0","0","0","0"],
   ["0","0","0","0","0","0","0","0","0","0"],
   ["0","0","0","0","0","0","0","0","0","0"],
   ["0","0","0","0","0","0","0","0","0","0"],
   ["0","0","0","0","0","0","0","0","0","0"],
   ["0","0","0","0","0","0","0","0","0","0"],
   ["0","0","0","0","0","0","0","0","0","0"],
   ["0","0","0","0","0","0","0","0","0","0"],
   ["0","0","0","0","0","0","0","0","0","0"],
   ["G","0","0","0","0","0","0","0","0","0"]
]

# ======================================================
# --- Helper: Convert layout to occupancy grid ---
# ======================================================

def make_occupancy_grid(layout):
    rows = len(layout)
    cols = len(layout[0])
    data = np.zeros((rows + 2, cols + 2), dtype=int)  # add boundary
    start = goal = None

    # Fill edges with walls
    data[0, :] = WALL_VALUE
    data[-1, :] = WALL_VALUE
    data[:, 0] = WALL_VALUE
    data[:, -1] = WALL_VALUE

    for r, row in enumerate(layout):
        for c, val in enumerate(row):
            rr, cc = r + 1, c + 1
            if val == "1":
                data[rr, cc] = WALL_VALUE
            elif val == "0":
                data[rr, cc] = FREE_VALUE
            elif val == "R":
                data[rr, cc] = FREE_VALUE
                start = (rr, cc)
            elif val == "G":
                data[rr, cc] = FREE_VALUE
                goal = (rr, cc)
    return data, start, goal

# ======================================================
# --- Static map publisher ---
# ======================================================

class StaticMapPublisher(Node):
    def __init__(self, grid_data):
        super().__init__('static_map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.grid_data = grid_data
        self.rows, self.cols = grid_data.shape
        self.resolution = GRID_RESOLUTION
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.cols
        msg.info.height = self.rows
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        flipped = np.flipud(self.grid_data)
        msg.data = flipped.flatten().tolist()
        self.publisher.publish(msg)
        self.get_logger().info_once("Publishing static /map to Nav2.")

# ======================================================
# --- Initial pose publisher ---
# ======================================================

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def set_pose(self, x, y, yaw_deg):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        yaw = math.radians(yaw_deg)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(msg)
        self.get_logger().info(f"Initial pose set → ({x:.2f}, {y:.2f}), yaw={yaw_deg:.1f}°")

# ======================================================
# --- Nav2 goal sender ---
# ======================================================

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw_deg=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        yaw = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)
        self.get_logger().info(f"Goal sent → ({x:.2f}, {y:.2f}), yaw={yaw_deg:.1f}°")

# ======================================================
# --- Utility: grid index → map coordinates ---
# ======================================================

def grid_to_map_xy(r, c, rows, cols, resolution):
    x = c * resolution
    y = (rows - 1 - r) * resolution
    return x, y

# ======================================================
# --- MAIN ---
# ======================================================

def main():
    rclpy.init()

    grid_data, start_idx, goal_idx = make_occupancy_grid(GRID_LAYOUT)
    if start_idx is None or goal_idx is None:
        print("❌ ERROR: Grid must have 'R' and 'G'")
        return

    map_pub = StaticMapPublisher(grid_data)
    pose_pub = InitialPosePublisher()
    nav2_client = Nav2Client()

    rows, cols = grid_data.shape
    start_x, start_y = grid_to_map_xy(start_idx[0], start_idx[1], rows, cols, GRID_RESOLUTION)
    goal_x, goal_y = grid_to_map_xy(goal_idx[0], goal_idx[1], rows, cols, GRID_RESOLUTION)

    print("\n--- GRID COORDINATE SUMMARY ---")
    print(f"Start grid cell: {start_idx} → map ({start_x:.2f}, {start_y:.2f})")
    print(f"Goal  grid cell: {goal_idx} → map ({goal_x:.2f}, {goal_y:.2f})")
    print("Grid rows ↑ (north), cols → (east)")
    print(f"Robot will start heading: {ROBOT_HEADING_DEG}°")
    print(f"Goal orientation: {GOAL_HEADING_DEG}°\n")

    # Wait for Nav2
    time.sleep(2.0)

    # Set initial robot pose (physical heading you choose)
    pose_pub.set_pose(start_x, start_y, ROBOT_HEADING_DEG)

    # Spin once to publish map
    rclpy.spin_once(map_pub, timeout_sec=1.0)
    time.sleep(1.0)

    # Send navigation goal with desired orientation
    nav2_client.send_goal(goal_x, goal_y, GOAL_HEADING_DEG)

    # Keep spinning
    while rclpy.ok():
        rclpy.spin_once(map_pub, timeout_sec=0.1)
        rclpy.spin_once(nav2_client, timeout_sec=0.1)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


# Part	Purpose
# GRID_LAYOUT	You define your map here ("1", "0", "R", "G")
# make_occupancy_grid()	Converts the layout into an occupancy grid and adds a 1-cell wall border
# StaticMapPublisher	Publishes /map every second to feed Nav2
# InitialPosePublisher	Sets the robot’s start position and orientation
# Nav2Client	Sends the navigation goal (goal position + orientation)
# grid_to_map_xy()	Converts grid indices to meters for Nav2


# 🧭 Orientation and placement

# You can physically place your robot in the direction you want (e.g., facing East).

# The code sets the orientation (yaw_deg) when calling:

# pose_pub.set_pose(start_x, start_y, 0.0)  # yaw_deg = 0 means East


# Change yaw_deg to 90 for North, 180 for West, -90 for South.


# 📏 Map Scaling

# Each cell = GRID_RESOLUTION meters (default 0.2 m).
# So your 6×5 layout (plus 2-cell border) = (8×7) grid = 1.4 m × 1.6 m map.
