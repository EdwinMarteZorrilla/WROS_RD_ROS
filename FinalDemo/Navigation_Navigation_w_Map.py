#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
import math
import time

# ======================================================
# --- INITIAL POSE PUBLISHER ---
# ======================================================
class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
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
        self.get_logger().info(f"📍 Initial pose set to x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°")


# ======================================================
# --- NAV2 CLIENT ---
# ======================================================
class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_done_callback)

        self.get_logger().info(f"🎯 Goal sent: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°")

    def goal_done_callback(self, future):
        self.get_logger().info("✅ Navigation goal completed. Stopping robot.")
        stop_msg = Twist()
        self.stop_pub.publish(stop_msg)
        self.get_logger().info("🛑 Robot stopped.")


# ======================================================
# --- MAIN ---
# ======================================================
def main():
    rclpy.init()

    # --- Create nodes ---
    pose_pub = InitialPosePublisher()
    nav2_client = Nav2Client()

    # --- Wait a bit for Nav2 to start ---
    time.sleep(3.0)

    # --- Set the initial pose of the robot on the map ---
    start_x = 1.0   # meters
    start_y = 0.5
    start_yaw = 0.0
    Yaw (deg)	Facing direction
# 0	East / +X
# 90	North / +Y
# 180	West / -X
# 270 / -90	South / -Y
    pose_pub.set_pose(start_x, start_y, start_yaw)
    time.sleep(1.0)

    # --- Send a navigation goal ---
    goal_x = 3.0
    goal_y = 2.0
    goal_yaw = 0.0
    # Yaw (deg)	Facing direction
# 0	East / +X
# 90	North / +Y
# 180	West / -X
# 270 / -90	South / -Y
    nav2_client.send_goal(goal_x, goal_y, goal_yaw)

    # --- Spin until navigation is complete ---
    try:
        while rclpy.ok():
            rclpy.spin_once(nav2_client, timeout_sec=0.1)
    except KeyboardInterrupt:
        nav2_client.get_logger().info("🛑 Shutdown requested by user.")

    # --- Shutdown ---
    nav2_client.destroy_node()
    pose_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
