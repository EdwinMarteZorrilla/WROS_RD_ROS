#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt

class ObjectCenteringNode(Node):
    def __init__(self):
        super().__init__('object_centering')

        # ROS publisher for rotation commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # CHANGE THIS IP TO YOUR MQTT BROKER
        self.mqtt_broker_ip = "127.0.0.1"
        self.mqtt_client.connect(self.mqtt_broker_ip, 1883, 60)
        self.mqtt_client.loop_start()

        # Subscribe to topic from Edge Impulse or detection node
        self.mqtt_client.subscribe("firevolx/object_detection")

        # PID-like rotation control parameters
        self.kp = 1.2                # proportional gain (higher = faster correction)
        self.max_angular_speed = 0.8 # maximum rotation speed
        self.center_threshold = 0.07 # acceptable error range to be considered centered

        # internal variables
        self.object_x = None
        self.centered = False

        self.get_logger().info("? Object Centering Node started and waiting for detections...")

    # MQTT connection
    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker at {self.mqtt_broker_ip}, code {rc}")

    # MQTT message handler
    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            if "x" in data:
                self.object_x = float(data["x"])
                self.centered = False
                self.get_logger().info(f"[MQTT] Object detection received: x={self.object_x:.3f}")
            else:
                self.get_logger().warn("[MQTT] Missing 'x' field in message.")
        except Exception as e:
            self.get_logger().warn(f"MQTT parse error: {e}")

    def rotate_to_center(self):
        if self.object_x is None:
            return

        # calculate how far object is from image center (0.5)
        error_x = self.object_x - 0.5
        twist = Twist()

        # compute angular speed
        if abs(error_x) > self.center_threshold:
            angular_z = -self.kp * error_x
            angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)

            self.get_logger().info(f"Rotating: error_x={error_x:+.3f}, angular_z={angular_z:+.3f}")
            self.get_logger().info(f"/cmd_vel ? angular.z={angular_z:+.3f}")

        else:
            # object centered ? stop rotation
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            if not self.centered:
                self.centered = True
                self.mqtt_client.publish("camera/center_done", json.dumps({"status": "centered"}))
                self.get_logger().info("? Object centered! Published 'camera/center_done'.")
            else:
                self.get_logger().info("Object remains centered.")

    def loop(self):
        timer_period = 0.2  # 5 Hz
        self.create_timer(timer_period, self.rotate_to_center)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectCenteringNode()
    node.loop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
