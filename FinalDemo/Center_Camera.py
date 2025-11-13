X#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import math
import time

# ==========================================================
# --- Robot Auto-Centering Node (no servo motion) ---
# ==========================================================

class ObjectCentering(Node):
    def __init__(self):
        super().__init__('object_centering')

        # --- Odometry for robot orientation ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.robot_yaw = 0.0

        # --- MQTT setup ---
        self.object_x = 0.5
        self.object_y = 0.5
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)  # adjust broker if needed
        self.mqtt_client.loop_start()

        # --- Timer to update rotation ---
        self.create_timer(0.05, self.update_motion)

        # --- CmdVelPublisher reference (set externally) ---
        self.cmd_node = None  # must set CmdVelPublisher externally

    # ======================================================
    # --- Odometry callback ---
    # ======================================================
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ======================================================
    # --- MQTT handling ---
    # ======================================================
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ Connected to MQTT broker")
        client.subscribe("edgeimpulse/alert")  # topic publishing "x,y"

    def on_mqtt_message(self, client, userdata, msg):
        try:
            x_str, y_str = msg.payload.decode().split(",")
            print("x_val:", x_val)
            print("y_val:", y_val)
            x_val = float(x_str)
            y_val = float(y_str)

            # --- Auto-normalization for 640x480 camera ---
            if x_val > 1.0 or y_val > 1.0:
                IMG_WIDTH = 640.0
                IMG_HEIGHT = 480.0
                x_val /= IMG_WIDTH
                y_val /= IMG_HEIGHT

            # Clamp to [0,1]
            self.object_x = max(0.0, min(1.0, x_val))
            self.object_y = max(0.0, min(1.0, y_val))

            self.get_logger().debug(f"📡 Object normalized: x={self.object_x:.2f}, y={self.object_y:.2f}")

        except Exception as e:
            self.get_logger().warn(f"⚠️ Invalid MQTT message format: {e}")

    # ======================================================
    # --- Main motion control (robot rotation only) ---
    # ======================================================
    def update_motion(self):
        # Error relative to image center (0.5, 0.5)
        error_x = self.object_x - 0.5

        # Threshold for “close enough” alignment
        threshold = 0.05

        # Only rotate if the object is significantly off-center
        if abs(error_x) > threshold and self.cmd_node is not None:
            # Proportional control — rotate robot to center target
            rotation_speed = -error_x * 0.6  # tune gain (negative to correct direction)
            self.cmd_node.publish_rotation(rotation_speed)
            self.get_logger().info(f"🔄 Rotating to center (error_x={error_x:.2f}, speed={rotation_speed:.2f})")
        else:
            if self.cmd_node is not None:
                self.cmd_node.stop_rotation()

# ==========================================================
# --- CmdVelPublisher (robot motion helper) ---
# ==========================================================

from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_rotation(self, angular_speed):
        msg = Twist()
        msg.angular.z = angular_speed
        self.pub.publish(msg)

    def stop_rotation(self):
        msg = Twist()
        msg.angular.z = 0.0
        self.pub.publish(msg)

# ==========================================================
# --- Main entry point ---
# ==========================================================

def main(args=None):
    rclpy.init(args=args)
    node = ObjectCentering()
    cmd_node = CmdVelPublisher()
    node.cmd_node = cmd_node

    try:
        node.get_logger().info("🤖 Object centering node started.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down, stopping robot.")
        cmd_node.stop_rotation()
    finally:
        node.destroy_node()
        cmd_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
