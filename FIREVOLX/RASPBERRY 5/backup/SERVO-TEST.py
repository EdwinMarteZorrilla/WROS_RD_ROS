import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
from time import time

class SprayServoController(Node):
    def __init__(self):
        super().__init__('spray_servo_controller')

        # --- Parameters ---
        self.declare_parameter('servo_id', 3)
        self.declare_parameter('angle_150', 700)   # servo position for ~150
        self.declare_parameter('angle_170', 850)   # servo position for ~170
        self.declare_parameter('neutral', 850)
        self.declare_parameter('cycle_speed', 0.3)  # seconds per motion step
        self.declare_parameter('motion_duration', 5.0)

        # --- Load parameters ---
        self.servo_id = self.get_parameter('servo_id').value
        self.angle_150 = self.get_parameter('angle_150').value
        self.angle_170 = self.get_parameter('angle_170').value
        self.neutral = self.get_parameter('neutral').value
        self.cycle_speed = self.get_parameter('cycle_speed').value
        self.motion_duration = self.get_parameter('motion_duration').value

        # --- Publisher ---
        self.publisher = self.create_publisher(SetPWMServoState,
                                               '/ros_robot_controller/pwm_servo/set_state', 10)

        # --- Control state ---
        self.is_up_position = False
        self.start_time = time()

        # --- Timer ---
        self.timer = self.create_timer(self.cycle_speed, self.timer_callback)

        self.get_logger().info(" ^=^r  Servo motion started between 150   ^`^s170  ")

    def set_servo_position(self, position):
        msg = SetPWMServoState()
        servo_state = PWMServoState()
        servo_state.id = [self.servo_id]
        servo_state.position = [position]
        servo_state.offset = [0]
        msg.state.append(servo_state)
        self.publisher.publish(msg)
        self.get_logger().info(f" ^f^r Servo moved to {position}")

    def timer_callback(self):
        elapsed = time() - self.start_time
        if elapsed > self.motion_duration:
            # Stop motion and center
            self.get_logger().info(" ^|^e Motion complete. Centering servo...")
            self.set_servo_position(self.neutral)
            self.timer.cancel()
            return

        # Alternate servo position
        if self.is_up_position:
            self.set_servo_position(self.angle_150)
        else:
            self.set_servo_position(self.angle_170)
        self.is_up_position = not self.is_up_position

def main(args=None):
    rclpy.init(args=args)
    node = SprayServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" ^z   ^o Interrupted by user. Centering servo...")
        node.set_servo_position(node.neutral)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


