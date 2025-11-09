#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
import numpy as np
import math
import time
from enum import Enum

# ======================================================
# --- USER SETTINGS ---
# ======================================================

GRID_RESOLUTION = 0.2    # meters per cell
WALL_VALUE = 100
FREE_VALUE = 0
UNKNOWN_VALUE = -1

# Robot heading in the real world when placed physically (East)
ROBOT_HEADING_DEG =  0.0

# Goal orientation at destination (e.g., North)
GOAL_HEADING_DEG = 90.0

# Define your grid layout ('R' = robot start, 'G' = goal)
GRID_LAYOUT = [
    ["R","0","0","0","0","0","0","0","0","0"],
    ["0","0","0","0","0","0","0","0","0","0"],
    ["0","0","0","0","0","0","0","0","G,"0"],
    ["0","0","0","0","0","0","0","0","0","0"],
    ["0","0","0","0","0","0","0","0","0","0"],
    ["0","0","0","0","0","0","0","0","0","0"],
    ["0","0","0","0","0","0","0","0","0","0"],
    ["0","0","0","0","0","0","0","0","0","0"],
    ["0","0","0","0","0","0","0","0","0","0"],
    ["0","0","0","0","0","0","0","0","0","0"]
]

# === NEW MISSION PARAMETERS ===
FORWARD_DISTANCE_1 = 0.5    # meters to move forward after initial goal
ROTATION_ANGLE_DEG = -90.0  # Rotate -90 degrees (right/clockwise)
FORWARD_DISTANCE_2 = 0.3    # meters to move closer to the object
# ==============================

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
# --- Static map publisher (No change) ---
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
        # Flip the grid to match ROS coordinate system (y up)
        flipped = np.flipud(self.grid_data)
        msg.data = flipped.flatten().tolist()
        self.publisher.publish(msg)
        self.get_logger().info_once("Publishing static /map to Nav2.")

# ======================================================
# --- Initial pose publisher (No change) ---
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
# --- Utility: grid index → map coordinates (Minor change to Yaw helper comment) ---
# ======================================================

def grid_to_map_xy(r, c, rows, cols, resolution):
    x = c * resolution
    # ROS y-axis points up, grid row index points down
    y = (rows - 1 - r) * resolution
    return x, y

def euler_to_quaternion(yaw_deg):
    """Converts a yaw angle (ROS convention: 0=East, CCW positive) to quaternion (z, w)."""
    yaw = math.radians(yaw_deg)
    # Roll and Pitch are 0 for 2D movement
    q = [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
    return q

def quaternion_to_yaw(q_z, q_w):
    # This is an approximation for 2D yaw, assuming q_x=q_y=0
    yaw_rad = 2 * math.atan2(q_z, q_w)
    return math.degrees(yaw_rad)

# ======================================================
# --- Mission State Machine ---
# ======================================================

class MissionState(Enum):
    """Defines the sequential steps of the mission."""
    INITIALIZING = 0
    GOTO_GOAL_1 = 1
    ACTION_MOVE_FORWARD_1 = 2
    ACTION_ROTATE_RIGHT = 3
    ACTION_CENTER_OBJECT = 4
    ACTION_MOVE_FORWARD_2 = 5
    GOTO_START_2 = 6
    FINISHED = 7

class MissionPlanner(Node):
    def __init__(self, start_pose, goal_pose, initial_heading, goal_heading):
        super().__init__('mission_planner')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

        # Store initial poses (x, y, yaw_deg)
        self.initial_pose = (start_pose[0], start_pose[1], initial_heading)
        self.goal_1_pose = (goal_pose[0], goal_pose[1], goal_heading)
        
        # State tracking
        self.mission_state = MissionState.INITIALIZING
        self.current_x = start_pose[0]
        self.current_y = start_pose[1]
        self.current_yaw_deg = initial_heading
        
        self.get_logger().info(f"Mission loaded. Start: ({start_pose[0]:.2f}, {start_pose[1]:.2f}), Goal: ({goal_pose[0]:.2f}, {goal_pose[1]:.2f})")
        self.timer = self.create_timer(0.5, self.mission_loop)

    def mission_loop(self):
        """Drives the state machine."""
        if self.mission_state == MissionState.INITIALIZING:
            self.mission_state = MissionState.GOTO_GOAL_1
            # Note: We must call send_goal, not start_navigation (typo fix in previous response)
            self.send_goal(self.goal_1_pose[0], self.goal_1_pose[1], self.goal_1_pose[2])
        
        # All other steps are triggered by the completion callback

    def send_goal(self, x, y, yaw_deg):
        """Constructs and sends a NavigateToPose goal."""
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("NavigateToPose Action Server not available!")
            return None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        q_z, q_w = euler_to_quaternion(yaw_deg)[2:]
        goal_msg.pose.pose.orientation.z = q_z
        goal_msg.pose.pose.orientation.w = q_w
        
        self.get_logger().info(f"Sending goal ({self.mission_state.name}) → ({x:.2f}, {y:.2f}), yaw={yaw_deg:.1f}°")
        
        # Store the target pose for the next step's calculation
        self.current_x = x
        self.current_y = y
        self.current_yaw_deg = yaw_deg

        # Send goal asynchronously
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when the server accepts or rejects the goal."""
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server!")
            return
        
        self.get_logger().info("Goal accepted. Waiting for result...")
        self.current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Called when the goal completes (success or failure)."""
        result = future.result().result
        status = future.result().status
        
        if status == 4: # GoalStatus.STATUS_SUCCEEDED (rclpy doesn't export enum easily)
            self.get_logger().info(f"Step {self.mission_state.name} succeeded!")
            self.advance_mission_state()
        else:
            self.get_logger().error(f"Step {self.mission_state.name} failed with status: {status}")
            self.mission_state = MissionState.FINISHED # Abort on failure

    def advance_mission_state(self):
        """Calculates the next target pose and moves to the next state."""
        current_pose = (self.current_x, self.current_y, self.current_yaw_deg)
        self.mission_state = MissionState(self.mission_state.value + 1)
        
        self.get_logger().info(f"\n--- Advancing to State: {self.mission_state.name} ---")

        if self.mission_state == MissionState.ACTION_MOVE_FORWARD_1:
            # 1. Move forward a distance
            x_next, y_next, yaw_next = self._calculate_relative_forward(
                current_pose, FORWARD_DISTANCE_1)
            self.send_goal(x_next, y_next, yaw_next)

        elif self.mission_state == MissionState.ACTION_ROTATE_RIGHT:
            # 2. Rotate right
            x_next, y_next, yaw_next = self._calculate_relative_rotation(
                current_pose, ROTATION_ANGLE_DEG)
            self.send_goal(x_next, y_next, yaw_next)
        
        elif self.mission_state == MissionState.ACTION_CENTER_OBJECT:
            # 3. Center on an object (Simulated step - applying a small corrective rotation)
            self.get_logger().warn("SIMULATING: Object centering (applying -5.0 degree corrective rotation).")
            
            # In a real system, vision processing would calculate the required delta_yaw_deg.
            CORRECTIVE_ROTATION_DEG = -5.0 
            
            x_next, y_next, yaw_next = self._calculate_relative_rotation(
                current_pose, CORRECTIVE_ROTATION_DEG)
            self.send_goal(x_next, y_next, yaw_next)


        elif self.mission_state == MissionState.ACTION_MOVE_FORWARD_2:
            # 4. Move forward until close enough to the object
            x_next, y_next, yaw_next = self._calculate_relative_forward(
                current_pose, FORWARD_DISTANCE_2)
            self.send_goal(x_next, y_next, yaw_next)

        elif self.mission_state == MissionState.GOTO_START_2:
            # 5. Go back to the initial start position
            # We use the initial yaw, as that's the pose we saved
            self.send_goal(self.initial_pose[0], self.initial_pose[1], self.initial_pose[2])
            
        elif self.mission_state == MissionState.FINISHED:
            self.get_logger().info("✅ Mission sequence complete!")
            # Stop the mission loop timer
            self.destroy_timer(self.timer)
            # Signal rclpy to shutdown
            rclpy.shutdown()

    def _calculate_relative_forward(self, current_pose, distance):
        """Calculates new pose after moving 'distance' forward."""
        x, y, yaw_deg = current_pose
        yaw_rad = math.radians(yaw_deg)
        x_new = x + distance * math.cos(yaw_rad)
        y_new = y + distance * math.sin(yaw_rad)
        return x_new, y_new, yaw_deg

    def _calculate_relative_rotation(self, current_pose, delta_yaw_deg):
        """Calculates new pose after rotating 'delta_yaw_deg'. Ensures yaw is normalized."""
        x, y, yaw_deg = current_pose
        yaw_new_deg = yaw_deg + delta_yaw_deg
        
        # Normalize yaw to be between -180 and 180 degrees
        while yaw_new_deg > 180:
            yaw_new_deg -= 360
        while yaw_new_deg <= -180:
            yaw_new_deg += 360
            
        return x, y, yaw_new_deg

# ======================================================
# --- MAIN EXECUTION ---
# ======================================================

def main():
    rclpy.init()

    grid_data, start_idx, goal_idx = make_occupancy_grid(GRID_LAYOUT)
    if start_idx is None or goal_idx is None:
        print("❌ ERROR: Grid must have 'R' and 'G'")
        return

    rows, cols = grid_data.shape
    start_x, start_y = grid_to_map_xy(start_idx[0], start_idx[1], rows, cols, GRID_RESOLUTION)
    goal_x, goal_y = grid_to_map_xy(goal_idx[0], goal_idx[1], rows, cols, GRID_RESOLUTION)

    print("\n--- MISSION SUMMARY ---")
    print(f"Initial Start Position (Map Coords): ({start_x:.2f}, {start_y:.2f}), Yaw: {ROBOT_HEADING_DEG}°")
    print(f"Goal 1 Target Position (Map Coords): ({goal_x:.2f}, {goal_y:.2f}), Yaw: {GOAL_HEADING_DEG}°")
    print(f"Return Start Position (Map Coords): ({start_x:.2f}, {start_y:.2f}), Yaw: {ROBOT_HEADING_DEG}°")
    print("---------------------------------\n")

    # 1. Start required ROS 2 nodes
    map_pub = StaticMapPublisher(grid_data)
    pose_pub = InitialPosePublisher()
    
    # 2. Initialize the state machine
    planner = MissionPlanner(
        start_pose=(start_x, start_y), 
        goal_pose=(goal_x, goal_y),
        initial_heading=ROBOT_HEADING_DEG,
        goal_heading=GOAL_HEADING_DEG
    )

    # 3. Wait for environment readiness (Simulated wait)
    time.sleep(2.0)

    # 4. Set initial robot pose
    pose_pub.set_pose(start_x, start_y, ROBOT_HEADING_DEG)
    
    # 5. Spin the map publisher once and wait for map to load
    rclpy.spin_once(map_pub, timeout_sec=1.0)
    time.sleep(1.0)
    
    # 6. Start the mission loop
    print("🚀 Starting Mission Sequence...")
    rclpy.spin(planner) # This replaces the while rclpy.ok() loop

if __name__ == '__main__':
    main()