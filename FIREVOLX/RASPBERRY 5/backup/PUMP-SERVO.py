import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
from time import time

class SprayServoController(Node):
    def _init_(self):
        super()._init_('spray_servo_controller')

        # --- Parameters ---
        self.declare_parameter('servo_id', 3)
        self.declare_parameter('angle_150', 750)    # servo position for ~150 degrees
        self.declare_parameter('angle_170', 850)    # servo position for ~170 degrees
        self.declare_parameter('neutral', 850)      # Neutral/stop position
        self.declare_parameter('motion_duration', 5.0) # Total duration for the cyclic motion
        
        # --- Smoothing Parameters ---
        # How much to change the servo position by in each time step (smaller = smoother/slower)
        self.declare_parameter('smooth_step', 2) 
        # Time interval for the movement timer (must be fast for smooth steps, e.g., 100Hz)
        self.declare_parameter('timer_period', 0.01) 

        # --- Load parameters ---
        self.servo_id = self.get_parameter('servo_id').value
        self.angle_150 = self.get_parameter('angle_150').value
        self.angle_170 = self.get_parameter('angle_170').value
        self.neutral = self.get_parameter('neutral').value
        self.motion_duration = self.get_parameter('motion_duration').value
        self.smooth_step = self.get_parameter('smooth_step').value
        self.timer_period = self.get_parameter('timer_period').value
        
        # --- Publisher ---
        self.publisher = self.create_publisher(SetPWMServoState,
                                               '/ros_robot_controller/pwm_servo/set_state', 10)

        # --- Control state for Smoothing ---
        # The actual current position of the servo
        self.current_position = float(self.neutral) 
        # The position the servo is trying to reach
        self.target_position = float(self.angle_170)
        # Tracks which endpoint we are currently aiming for in the cycle
        self.is_moving_to_150 = False 
        self.start_time = time()

        # --- Timer ---
        # Use the faster period for smooth, incremental updates
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(" ^=^r  Servo smooth motion started between 150 and 170 degrees.")

    def set_servo_position(self, position):
        """Publishes the command to set the servo's position."""
        msg = SetPWMServoState()
        servo_state = PWMServoState()
        servo_state.id = [self.servo_id]
        
        # Position must be an integer for servo commands
        servo_state.position = [int(round(position))] 
        
        servo_state.offset = [0]
        msg.state.append(servo_state)
        self.publisher.publish(msg)
        # self.get_logger().info(f" ^f^r Servo moved to {int(round(position))}") # Uncomment for debugging

    def timer_callback(self):
        elapsed = time() - self.start_time
        
        # 1. Check if the total motion duration has been exceeded
        if elapsed > self.motion_duration:
            self.get_logger().info(" ^|^e Motion complete. Moving to neutral...")
            self.target_position = float(self.neutral)
            
            # Check if we've arrived at neutral
            if abs(self.current_position - self.target_position) <= self.smooth_step:
                self.set_servo_position(self.neutral)
                self.timer.cancel()
                self.get_logger().info(" ^z  ^o Servo motion timer cancelled.")
                return

        # 2. Determine the target position if the motion is still active
        elif self.is_moving_to_150:
            self.target_position = float(self.angle_150)
        else:
            self.target_position = float(self.angle_170)
            
        # 3. Step the current position towards the target position
        
        # Calculate the difference and direction
        difference = self.target_position - self.current_position
        
        if abs(difference) > self.smooth_step:
            # If far from target, move one step in the correct direction
            step = self.smooth_step if difference > 0 else -self.smooth_step
            self.current_position += step
        else:
            # If close to target, snap to the target and switch direction for the next cycle
            self.current_position = self.target_position
            if elapsed <= self.motion_duration: # Only flip the target if we're still in the main cycle
                self.is_moving_to_150 = not self.is_moving_to_150
                self.get_logger().info(f"Target {int(round(self.target_position))} reached. Switching to {'150' if self.is_moving_to_150 else '170'}.")
            
        # 4. Apply the new position
        self.set_servo_position(self.current_position)

# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SprayServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" ^z  ^o Interrupted by user. Smoothly moving to neutral...")
        # Emergency stop logic: move to neutral before shutting down
        node.target_position = float(node.neutral)
        
        # Simple loop to smoothly move to neutral upon Ctrl+C
        while abs(node.current_position - node.neutral) > node.smooth_step:
            step = node.smooth_step if node.neutral > node.current_position else -node.smooth_step
            node.current_position += step
            node.set_servo_position(node.current_position)
            # Short sleep is necessary here since we're outside the rclpy spinning
            import time as pytime
            pytime.sleep(node.timer_period) 

        node.set_servo_position(node.neutral) # Final snap to neutral
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if _name_ == "_main_":
    main()