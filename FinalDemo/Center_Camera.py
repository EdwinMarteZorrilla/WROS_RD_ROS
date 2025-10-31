#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
from sensor_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import math
import time

# ---------------- Servo Control Node ----------------
class ServoTracker(Node):
    def __init__(self):
        super().__init__('servo_tracker')

        # --- Servo parameters (from your working program) ---
        self.declare_parameter('servo_vertical_id', 1)
        self.declare_parameter('vertical_angle_min', 1300)
        self.declare_parameter('vertical_angle_max', 1800)
        self.declare_parameter('vertical_neutral', 1500)

        self.declare_parameter('servo_horizontal_id', 2)
        self.declare_parameter('horizontal_angle_min', 1050)
        self.declare_parameter('horizontal_angle_max', 1950)
        self.declare_parameter('horizontal_neutral', 1500)

        # Load parameters
        self.vertical_id = self.get_parameter('servo_vertical_id').value
        self.vert_min = self.get_parameter('vertical_angle_min').value
        self.vert_max = self.get_parameter('vertical_angle_max').value
        self.vert_neutral = self.get_parameter('vertical_neutral').value

        self.horiz_id = self.get_parameter('servo_horizontal_id').value
        self.horiz_min = self.get_parameter('horizontal_angle_min').value
        self.horiz_max = self.get_parameter('horizontal_angle_max').value
        self.horiz_neutral = self.get_parameter('horizontal_neutral').value

        # Current servo positions
        self.vertical_pos = float(self.vert_neutral)
        self.horizontal_pos = float(self.horiz_neutral)

        # Proportional gains for servo adjustment
        self.K_pan = 300
        self.K_tilt = 300

        # --- Publisher ---
        self.pub = self.create_publisher(SetPWMServoState,
                                         '/ros_robot_controller/pwm_servo/set_state', 10)

        # --- Odometry for robot rotation ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.robot_yaw = 0.0

        # --- CmdVelPublisher reference (set externally) ---
        self.cmd_node = None  # must set CmdVelPublisher node

        # --- MQTT setup ---
        self.object_x = 0.5
        self.object_y = 0.5
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect("localhost", 1883, 60)  # adjust broker IP
        self.mqtt_client.loop_start()

        # --- Timer to update servo + robot ---
        self.create_timer(0.05, self.update_motion)

    # ---------------- Odometry callback ----------------
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ---------------- MQTT callbacks ----------------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("object/coordinates")  # topic with normalized x,y

    def on_mqtt_message(self, client, userdata, msg):
        # Expect message "x,y" normalized 0-1
        try:
            x_str, y_str = msg.payload.decode().split(",")
            self.object_x = float(x_str)
            self.object_y = float(y_str)
        except:
            self.get_logger().warn("Invalid MQTT message format")

    # ---------------- Servo command ----------------
    def set_servo_position(self, servo_id, position):
        msg = SetPWMServoState()
        servo_state = PWMServoState()
        servo_state.id = [servo_id]
        servo_state.position = [int(round(position))]
        servo_state.offset = [0]
        msg.state.append(servo_state)
        self.pub.publish(msg)

    # ---------------- Update motion ----------------
    def update_motion(self):
        # --- Compute servo targets ---
        error_x = self.object_x - 0.5
        error_y = self.object_y - 0.5

        target_horiz = self.horiz_neutral + self.K_pan * error_x
        target_vert = self.vert_neutral - self.K_tilt * error_y

        # Clamp to limits
        target_horiz = max(self.horiz_min, min(self.horiz_max, target_horiz))
        target_vert = max(self.vert_min, min(self.vert_max, target_vert))

        # Smooth move (simple proportional step)
        step = 10
        if abs(target_horiz - self.horizontal_pos) > step:
            self.horizontal_pos += step if target_horiz > self.horizontal_pos else -step
        else:
            self.horizontal_pos = target_horiz

        if abs(target_vert - self.vertical_pos) > step:
            self.vertical_pos += step if target_vert > self.vertical_pos else -step
        else:
            self.vertical_pos = target_vert

        # Send to servo
        self.set_servo_position(self.horiz_id, self.horizontal_pos)
        self.set_servo_position(self.vertical_id, self.vertical_pos)

        # --- Robot rotation if object too far left/right ---
        if self.cmd_node is not None:
            threshold = 0.1
            if abs(error_x) > threshold:
                delta_yaw = error_x * math.radians(30)  # proportional yaw adjustment
                target_yaw = self.robot_yaw + delta_yaw
                self.cmd_node.rotate_to_yaw(target_yaw, odom_sub=self.odom_sub, yaw_tol=0.02, max_speed=0.3)

# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = ServoTracker()

    # --- You must create CmdVelPublisher and assign ---
    from your_previous_code import CmdVelPublisher
    cmd_node = CmdVelPublisher()
    node.cmd_node = cmd_node

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down, centering servos...")
        node.set_servo_position(node.vertical_id, node.vert_neutral)
        node.set_servo_position(node.horiz_id, node.horiz_neutral)
    finally:
        node.destroy_node()
        cmd_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
