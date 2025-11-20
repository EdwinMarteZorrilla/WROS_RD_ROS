#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import math
from rclpy.executors import MultiThreadedExecutor
import json

# ==========================================================
# --- Robot Auto-Centering Node (Improved) ---
# ==========================================================

class ObjectCentering(Node):
    def __init__(self):
        super().__init__('object_centering')

        # --------------------------------------------
        # CONFIGURATION (adjust as needed)
        # --------------------------------------------

        # tracking
        self.low_threshold = 0.02        # ignore tiny errors
        self.high_threshold = 0.07       # must exceed this to move
        self.rotation_gain = 2        # rotation multiplier
        self.smoothing = 0.25            # 0.1–0.3 recommended

        # motor protections
        self.min_start_speed = 0.95      # torque to overcome friction
        self.max_speed = 1.0             # cap rotation
        # self.pulse_time = 0.20           # duration of rotation burst
        # self.cooldown_after_pulses = 10  # pulses before cooling break
        # self.cooldown_time = 0.8         # seconds to rest motors

        # --------------------------------------------
        # INTERNAL STATE
        # --------------------------------------------
        self.object_x = 0.5
        self.filtered_x = 0.5

        # self.is_pulsing = False
        # # self.pulse_end_time = 0.0

        # # self.pulse_count = 0
        # self.cooldown_until = 0.0

        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # Timer
        self.create_timer(0.05, self.update_motion)

        self.cmd_node = None


    # --------------------------------------------
    # MQTT callbacks
    # --------------------------------------------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("edgeimpulse/alert")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())

            dets = data.get("detections", [])
            if not dets:
                return

            det = dets[0]
            x_val = float(det.get("x", 0)) / 96.0
            x_val = max(0.0, min(1.0, x_val))

            # low-pass filter (smooths noise)
            self.filtered_x = (
                self.filtered_x * (1 - self.smoothing)
                + x_val * self.smoothing
            )

        except Exception as e:
            self.get_logger().warn(f"MQTT parse error: {e}")


    # --------------------------------------------
    # CONTROL LOOP
    # --------------------------------------------
    
    
    def update_motion(self):
        if self.cmd_node is None:
            return

        error = self.filtered_x - 0.5
        abs_error = abs(error)
        
        # ----------------------------------------
        # 1. Deadband/Stop Logic
        # ----------------------------------------
        # Stop the robot if the error is within the deadband
        if abs_error < self.high_threshold:
            self.cmd_node.stop_rotation() # <-- Now we stop if we're centered
            return
        
        # ----------------------------------------
        # 2. Compute speed from error (P-Control)
        # ----------------------------------------
        speed_raw = -error * self.rotation_gain   # Base speed
        speed = speed_raw

        # ----------------------------------------
        # 3. Apply Minimum Torque (0.95)
        # ----------------------------------------
        # If the calculated speed is non-zero, but too low to move (below 0.95)
        if abs_error > self.low_threshold and abs(speed_raw) < self.min_start_speed:
            # Set speed to the minimum required torque (0.95)
            speed = self.min_start_speed * (1 if speed_raw > 0 else -1)
        
        # ----------------------------------------
        # 4. Cap max speed (1.0)
        # ----------------------------------------
        speed = max(-self.max_speed, min(self.max_speed, speed))

        # ----------------------------------------
        # 5. Publish continuous speed
        # ----------------------------------------
        self.cmd_node.publish_rotation(speed)

        self.get_logger().info(
            f"Continuous: error={error:.2f}, speed={speed:.2f}"
        )
        
        
        
    # def update_motion(self):
        # if self.cmd_node is None:
            # return

        # now = self.get_clock().now().nanoseconds / 1e9
        # error = self.filtered_x - 0.5

        # # ----------------------------------------
        # # 1. Currently pulsing → check if done
        # # ----------------------------------------
        # if self.is_pulsing:
            # if now >= self.pulse_end_time:
                # self.cmd_node.stop_rotation()
                # self.is_pulsing = False
            # return

        # # ----------------------------------------
        # # 2. Motor cooldown
        # # ----------------------------------------
        # if now < self.cooldown_until:
            # return

        # # ----------------------------------------
        # # 3. Deadband: ignore tiny errors
        # # ----------------------------------------
        # if abs(error) < self.low_threshold:
            # return

        # # ----------------------------------------
        # # 4. Only act if error is significant
        # # ----------------------------------------
        # if abs(error) < self.high_threshold:
            # return

        # # ----------------------------------------
        # # 5. Compute speed from error
        # # ----------------------------------------
        # speed = -error * self.rotation_gain   # invert if needed

        # # minimum torque (fix right-turn problem)
        # if abs(speed) < self.min_start_speed:
            # speed = self.min_start_speed * (1 if speed > 0 else -1)

        # # cap max speed
        # speed = max(-self.max_speed, min(self.max_speed, speed))

        # # ----------------------------------------
        # # 6. Fire a short pulse
        # # ----------------------------------------
        # self.cmd_node.publish_rotation(speed)
        # self.is_pulsing = True
        # self.pulse_end_time = now + self.pulse_time

        # self.pulse_count += 1

        # # if many pulses → cooldown
        # if self.pulse_count >= self.cooldown_after_pulses:
            # self.cooldown_until = now + self.cooldown_time
            # self.pulse_count = 0

        # self.get_logger().info(
            # f"Pulse: error={error:.2f}, speed={speed:.2f}"
        # )


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
