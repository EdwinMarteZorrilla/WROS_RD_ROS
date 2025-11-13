#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import math
from rclpy.executors import MultiThreadedExecutor

# ==========================================================
# --- Robot Auto-Centering Node (Improved) ---
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
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # --- Timer to update rotation ---
        self.create_timer(0.05, self.update_motion)

        # --- CmdVelPublisher reference (set externally) ---
        self.cmd_node = None

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ Connected to MQTT broker")
        client.subscribe("object/coordinates")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            x_str, y_str = msg.payload.decode().split(",")
            x_val = float(x_str)
            y_val = float(y_str)

            # Normalize 96x96 pixel detections
            if x_val > 1.0 or y_val > 1.0:
                x_val /= 96.0
                y_val /= 96.0

            self.object_x = max(0.0, min(1.0, x_val))
            self.object_y = max(0.0, min(1.0, y_val))

        except Exception as e:
            self.get_logger().warn(f"⚠️ Invalid MQTT message: {e}")

    def update_motion(self):
        if self.cmd_node is None:
            return

        error_x = self.object_x - 0.5
        threshold = 0.02   #estaba en 0.05

        if abs(error_x) > threshold:
            rotation_speed = -error_x * 1.2  # tune this for direction  (ganancia 0.6)
            self.cmd_node.publish_rotation(rotation_speed)
            self.get_logger().info(f"🔄 Rotating to center (error_x={error_x:.2f}, speed={rotation_speed:.2f})")
        else:
            self.cmd_node.stop_rotation()

# ==========================================================
# --- CmdVelPublisher ---
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
# --- Main ---
# ==========================================================

def main(args=None):
    rclpy.init(args=args)
    node = ObjectCentering()
    cmd_node = CmdVelPublisher()
    node.cmd_node = cmd_node

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(cmd_node)

    try:
        node.get_logger().info("🤖 Object centering node started.")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down, stopping robot.")
        cmd_node.stop_rotation()
    finally:
        executor.shutdown()
        node.destroy_node()
        cmd_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
