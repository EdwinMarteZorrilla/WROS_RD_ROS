#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist
import math

# ==========================================================
# --- Robot Follow Object Node (forward + centering) ---
# ==========================================================

class FollowObject(Node):
    def __init__(self):
        super().__init__('follow_object')

        # --- Motion publisher ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Object position (normalized 0–1) ---
        self.object_x = 0.5
        self.object_y = 0.5

        # --- MQTT setup ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # --- Control loop (20 Hz) ---
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("🤖 FollowObject node initialized.")

    # ======================================================
    # --- MQTT Handlers ---
    # ======================================================
    def on_mqtt_connect(self, client, userdata, flags, rc):
        client.subscribe("object/coordinates")
        self.get_logger().info("✅ Subscribed to 'object/coordinates'")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            x_str, y_str = msg.payload.decode().split(",")
            x_val = float(x_str)
            y_val = float(y_str)

            # Normalize automatically if in pixels (640x480)
            if x_val > 1.0 or y_val > 1.0:
                x_val /= 640.0
                y_val /= 480.0

            # Clamp
            self.object_x = max(0.0, min(1.0, x_val))
            self.object_y = max(0.0, min(1.0, y_val))

        except Exception as e:
            self.get_logger().warn(f"⚠️ Bad MQTT format: {e}")

    # ======================================================
    # --- Motion Controller ---
    # ======================================================
    def control_loop(self):
        msg = Twist()

        # Compute horizontal offset (center = 0.5)
        error_x = self.object_x - 0.5
        threshold_x = 0.05

        # Keep object centered
        if abs(error_x) > threshold_x:
            msg.angular.z = -error_x * 0.8  # tune gain
        else:
            msg.angular.z = 0.0

        # Move forward until object is 2/3 down the screen (y >= 0.66)
        if self.object_y < 0.66:
            msg.linear.x = 0.15 * (1.0 - self.object_y)  # slow down as it approaches
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info_throttle(2.0, "🛑 Object close — stopping.")

        self.cmd_pub.publish(msg)

# ==========================================================
# --- Main Entry ---
# ==========================================================

def main(args=None):
    rclpy.init(args=args)
    node = FollowObject()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down follow node.")
    finally:
        node.cmd_pub.publish(Twist())  # stop robot
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
