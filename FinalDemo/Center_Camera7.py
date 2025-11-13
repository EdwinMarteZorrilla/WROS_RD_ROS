#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import math
import json
import time

# ==========================================================
# --- Robot Auto-Centering Node (MQTT JSON from EdgeImpulse) ---
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
        self.new_detection = False  # flag: new data ready to process
        self.processing = False     # flag: avoid overlapping actions

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # --- Timer to check motion ---
        self.create_timer(0.1, self.update_motion)  # every 100 ms

        # --- CmdVelPublisher reference ---
        self.cmd_node = None

    # ======================================================
    # --- Odometry callback ---
    # ======================================================
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ======================================================
    # --- MQTT Handling ---
    # ======================================================
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ Connected to MQTT broker")
        client.subscribe("edgeimpulse/alert")

    def on_mqtt_message(self, client, userdata, msg):
        # Ignore if already processing (avoid overwriting)
        if self.processing:
            return

        try:
            payload = json.loads(msg.payload.decode())
            detections = payload.get("detections", [])

            if not detections:
                self.get_logger().warn("⚠️ No detections found in message.")
                return

            det = detections[0]
            x_val = float(det.get("x", 0.0))
            y_val = float(det.get("y", 0.0))

            # Normalize coordinates if in pixels
            IMG_WIDTH = 96.0
            IMG_HEIGHT = 96.0
            if x_val > 1.0 or y_val > 1.0:
                x_val /= IMG_WIDTH
                y_val /= IMG_HEIGHT

            # Clamp to [0,1]
            self.object_x = max(0.0, min(1.0, x_val))
            self.object_y = max(0.0, min(1.0, y_val))
            self.new_detection = True  # mark as ready to process

            self.get_logger().info(
                f"🎯 Detection received: {det.get('label')} | x={self.object_x:.2f}, y={self.object_y:.2f}"
            )

        except Exception as e:
            self.get_logger().warn(f"⚠️ Invalid MQTT message format: {e}")

    # ======================================================
    # --- Motion control (robot rotation only) ---
    # ======================================================
    def update_motion(self):
        if not self.new_detection or self.processing or self.cmd_node is None:
            return

        self.processing = True  # lock new detections
        self.new_detection = False

        error_x = self.object_x - 0.5
        threshold = 0.05

        if abs(error_x) > threshold:
            rotation_speed = -error_x * 0.6
            self.cmd_node.publish_rotation(rotation_speed)
            self.get_logger().info(f"🔄 Rotating to center (error_x={error_x:.2f}, speed={rotation_speed:.2f})")

            # Let the rotation execute for 3 seconds before next detection
            time.sleep(3.0)
            self.cmd_node.stop_rotation()
            self.get_logger().info("✅ Rotation complete, ready for next detection.")

        else:
            self.cmd_node.stop_rotation()
            self.get_logger().info("✅ Object already centered.")

        self.processing = False  # unlock after done

# ==========================================================
# --- CmdVelPublisher (robot motion helper) ---
# ==========================================================

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
        node.get_logger().info("🚀 Object centering node started. Waiting for detections...")
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
#x
