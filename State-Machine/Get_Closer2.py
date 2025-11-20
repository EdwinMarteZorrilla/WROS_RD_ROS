#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json
import time
from rclpy.executors import MultiThreadedExecutor

# ==========================================================
# --- Robot Auto-Forward Node (Y-axis, precise approach) ---
# ==========================================================

class ObjectApproach(Node):
    def __init__(self):
        super().__init__('object_approach')

        # --- Object position (normalized 0–1) ---
        self.object_y = 0.5
        self.received_detection = False

        # --- Linear movement parameters ---
        self.target_y = 0.66  # valor Y normalizado donde queremos detenernos (aprox. 15cm)
        self.threshold_y = 0.02  # tolerancia para detenerse
        self.max_speed = 0.25
        self.gain = 1.2  # ganancia proporcional como en ObjectCentering

        # --- MQTT setup ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # --- Timer para actualizar movimiento ---
        self.create_timer(0.05, self.update_motion)

        # --- CmdVelPublisher referencia (externa) ---
        self.cmd_node = None

        # --- Logging throttle ---
        self._last_log_time = 0.0
        self._log_interval = 0.5

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ Connected to MQTT broker")
        client.subscribe("edgeimpulse/alert")  # tópico del transmitter

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            detections = payload.get("detections", [])

            if len(detections) == 0:
                self.get_logger().warn("⚠️ No detections found in MQTT message")
                return

            det = detections[0]
            y_val = float(det.get("y", 0))

            # Normalizar desde 96x96
            y_val /= 96.0
            self.object_y = max(0.0, min(1.0, y_val))

            self.received_detection = True

            current_time = time.time()
            if current_time - self._last_log_time > self._log_interval:
                self.get_logger().info(f"📡 Object detected: y={self.object_y:.2f}")
                self._last_log_time = current_time

        except json.JSONDecodeError:
            self.get_logger().warn("⚠️ Invalid MQTT message: not valid JSON")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error parsing MQTT message: {e}")

    def update_motion(self):
        if self.cmd_node is None or not self.received_detection:
            return

        error_y = self.target_y - self.object_y

        if error_y > self.threshold_y:
            # velocidad proporcional
            speed = min(error_y * self.gain, self.max_speed)
            self.cmd_node.publish_forward(speed)
            current_time = time.time()
            if current_time - self._last_log_time > self._log_interval:
                self.get_logger().info(f"⬆️ Moving forward (error_y={error_y:.2f}, speed={speed:.2f})")
                self._last_log_time = current_time
        else:
            self.cmd_node.stop_motion()
            current_time = time.time()
            if current_time - self._last_log_time > self._log_interval:
                self.get_logger().info(f"🛑 Reached target distance. Stopping (y={self.object_y:.2f})")
                self._last_log_time = current_time

# ==========================================================
# --- CmdVelPublisher ---
# ==========================================================

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_forward(self, linear_speed):
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def stop_motion(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

# ==========================================================
# --- Main ---
# ==========================================================

def main(args=None):
    rclpy.init(args=args)
    node = ObjectApproach()
    cmd_node = CmdVelPublisher()
    node.cmd_node = cmd_node

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(cmd_node)

    try:
        node.get_logger().info("🤖 Object approach (Y-axis, precise) node started.")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down, stopping robot.")
        cmd_node.stop_motion()
    finally:
        executor.shutdown()
        node.destroy_node()
        cmd_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
