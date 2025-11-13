#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json
import time
import math

class ObjectCentering(Node):
    def __init__(self):
        super().__init__('object_centering')

        # robot yaw from odometry
        self.robot_yaw = 0.0

        # last detection timestamp (seconds)
        self.last_detection_time = 0.0

        # normalized object position in image [0..1]
        self.object_x = 0.5
        self.object_y = 0.5

        # subscribe odom
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # MQTT: subscribe edgeimpulse/alert for detections
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # motion update timer
        self.timer_period = 0.05  # 20 Hz
        self.create_timer(self.timer_period, self.update_motion)

        # cmd_vel publisher wrapper node (used to publish /cmd_vel)
        self.cmd_node = CmdVelPublisher()

        # tuning
        self.detection_timeout = 0.5    # seconds without detections -> stop
        self.center_threshold = 0.05    # how close to center counts as centered
        self.kp = 0.6                   # proportional gain
        self.max_angular_speed = 0.4    # clamp angular.z (rad/s)

        self.get_logger().info("Object centering node started and MQTT client running.")

    # ---------- odometry ----------
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        # yaw calculation (assuming quaternion)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    # ---------- MQTT ----------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connected ({rc}) - subscribing to edgeimpulse/alert")
        client.subscribe("edgeimpulse/alert")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            detections = payload.get("detections", [])
            if not detections:
                return

            d = detections[0]  # use first detection

            # center of bbox
            cx = d.get("x", 0) + d.get("width", 0) / 2.0
            cy = d.get("y", 0) + d.get("height", 0) / 2.0

            # normalize using 640x480 (same as your pipeline)
            self.object_x = max(0.0, min(1.0, cx / 640.0))
            self.object_y = max(0.0, min(1.0, cy / 480.0))

            self.last_detection_time = time.time()

            self.get_logger().info(f"Detection update: x={self.object_x:.3f}, y={self.object_y:.3f}")

        except Exception as e:
            self.get_logger().warn(f"MQTT parse error: {e}")

    # ---------- motion logic ----------
    def update_motion(self):
        now = time.time()

        # if no recent detection -> stop rotation
        if (now - self.last_detection_time) > self.detection_timeout:
            # ensure robot is not rotating
            self.cmd_node.stop_rotation()
            return

        # compute error from image center
        error_x = self.object_x - 0.5

        if abs(error_x) <= self.center_threshold:
            # considered centered
            self.cmd_node.stop_rotation()
            return

        # proportional control with clamp
        rotation_speed = -error_x * self.kp
        # clamp
        if rotation_speed > 0:
            rotation_speed = min(rotation_speed, self.max_angular_speed)
        else:
            rotation_speed = max(rotation_speed, -self.max_angular_speed)

        # publish
        self.cmd_node.publish_rotation(rotation_speed)
        self.get_logger().info(f"Rotating (error_x={error_x:.3f}, speed={rotation_speed:.3f})")


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_pub_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_rotation(self, speed):
        msg = Twist()
        msg.angular.z = float(speed)
        self.pub.publish(msg)

    def stop_rotation(self):
        msg = Twist()
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectCentering()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down center node.")
        node.cmd_node.stop_rotation()
    finally:
        node.destroy_node()
        node.cmd_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
