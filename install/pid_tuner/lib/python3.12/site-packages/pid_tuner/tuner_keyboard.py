import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import sys
import termios
import tty
import threading
import time

# --- Instructions for the User ---
instructions = """
---------------------------------
Mecanum PID Keyboard Tuner
---------------------------------
- Includes Y-axis (strafing) control.
- All movements include a smooth ramp-up and ramp-down.

--- Movement (customizable duration/ramp) ---
   i: Move Straight Forward
   ,: Move Straight Backward
   j: Rotate Left (in place)
   l: Rotate Right (in place)
   a: Strafe Left
   d: Strafe Right
   
   k: STOP ROBOT NOW

--- Servo Control ---
   1: Set servo to 0 degrees
   2: Set servo to 90 degrees
   3: Set servo to 135 degrees (center)
   4: Set servo to 180 degrees
   5: Set servo to 270 degrees

---------------------------
Press Ctrl+C to quit
---------------------------
"""

# --- Key-to-Command Mapping ---
# THIS IS THE PART YOU CUSTOMIZE!
# Format for Twist: ('twist', linear_x, linear_y, angular_z, duration, ramp_time)
#   - duration: Total time for the action, including ramps.
#   - ramp_time: Time to accelerate from 0 to target, and to decelerate back to 0.
# Format for Servo: ('servo', angle_in_degrees)
key_bindings = {
    # Movement commands
    'w': ('twist', 0.7, 0.0, 0.0, 4.0, 0.5),
    's': ('twist', -0.7, 0.0, 0.0, 4.0, 0.5), 
    'a': ('twist', 0.0, 0.0, 3.0, 2.5, 0.0),  
    'd': ('twist', 0.0, 0.0, -3.0, 2.5, 0.0), 
    'j': ('twist', 0.0, 0.7, 0.0, 4.0, 0.0),
    'l': ('twist', 0.0, -0.7, 0.0, 4.0, 0.0),
    
    # Stop command (duration and ramp are ignored)
    ' ': ('twist', 0.0, 0.0, 0.0, 0.0, 0.0),

    # Servo commands
    '1': ('servo', 135.0),
    '2': ('servo', 270.0)
}


class KeyboardTunerNode(Node):
    def __init__(self):
        super().__init__('pid_keyboard_tuner_node')
        
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_publisher = self.create_publisher(Float32, '/servo_angle', 10)
        
        # Threading control
        self.movement_thread = None
        self.stop_event = threading.Event()

        self.get_logger().info("Mecanum Keyboard Tuner Node is ready.")

    def get_key(self):
        """Function to get a single key press from the terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def ramped_twist_worker(self, target_x, target_y, target_z, duration, ramp_time):
        """
        Worker function executed in a thread to handle ramped movement.
        """
        self.get_logger().info(
            f"Executing Ramped Twist: x={target_x}, y={target_y}, z={target_z} "
            f"for {duration}s with {ramp_time}s ramp."
        )
        
        start_time = time.time()
        # The frequency at which we publish velocity updates
        publish_frequency = 50.0  # 50 Hz
        
        while True:
            elapsed_time = time.time() - start_time
            
            # Check if the thread should stop or if time is up
            if self.stop_event.is_set() or elapsed_time > duration:
                break

            # --- Ramping Logic ---
            current_x, current_y, current_z = 0.0, 0.0, 0.0
            
            # 1. Ramp-up phase
            if elapsed_time < ramp_time:
                ramp_factor = elapsed_time / ramp_time
                current_x = target_x * ramp_factor
                current_y = target_y * ramp_factor
                current_z = target_z * ramp_factor
            # 2. Ramp-down phase
            elif elapsed_time > (duration - ramp_time):
                time_left = duration - elapsed_time
                ramp_factor = time_left / ramp_time
                # Ensure ramp_factor doesn't go negative due to timing inaccuracies
                ramp_factor = max(0, ramp_factor)
                current_x = target_x * ramp_factor
                current_y = target_y * ramp_factor
                current_z = target_z * ramp_factor
            # 3. Constant velocity phase
            else:
                current_x = target_x
                current_y = target_y
                current_z = target_z

            # Publish the calculated velocity
            twist_msg = Twist()
            twist_msg.linear.x = current_x
            twist_msg.linear.y = current_y
            twist_msg.angular.z = current_z
            self.twist_publisher.publish(twist_msg)
            
            time.sleep(1.0 / publish_frequency)

        # --- Final Stop Command ---
        # Ensure the robot is stopped when the loop finishes or is interrupted
        self.twist_publisher.publish(Twist()) # Publishes a zero-velocity Twist
        self.get_logger().info("Movement finished. STOP command sent.")

    def run_twist_command(self, linear_x, linear_y, angular_z, duration, ramp_time):
        """
        Stops any existing movement and starts a new ramped movement in a thread.
        """
        # If another movement is already in progress, signal it to stop
        if self.movement_thread is not None and self.movement_thread.is_alive():
            self.stop_event.set()
            self.movement_thread.join() # Wait for the old thread to finish

        # If the command is just to stop, we don't need a new thread
        if duration == 0.0:
            self.twist_publisher.publish(Twist())
            self.get_logger().info("Immediate STOP command sent.")
            return

        # Clear the stop event flag for the new thread
        self.stop_event.clear()
        
        # Create and start the new movement thread
        self.movement_thread = threading.Thread(
            target=self.ramped_twist_worker,
            args=(linear_x, linear_y, angular_z, duration, ramp_time)
        )
        self.movement_thread.start()

    def run_servo_command(self, angle):
        """Publishes a Float32 command to the servo topic."""
        self.get_logger().info(f"Executing Servo command: angle={angle:.1f} degrees")
        servo_msg = Float32()
        servo_msg.data = float(angle)
        self.servo_publisher.publish(servo_msg)

    def run(self):
        """The main loop of the node."""
        print(instructions)
        while rclpy.ok():
            key = self.get_key()

            if key == '\x03': # Ctrl+C
                break

            if key in key_bindings:
                command = key_bindings[key]
                cmd_type = command[0]

                if cmd_type == 'twist':
                    _, linear_x, linear_y, angular_z, duration, ramp_time = command
                    self.run_twist_command(linear_x, linear_y, angular_z, duration, ramp_time)
                elif cmd_type == 'servo':
                    _, angle = command
                    self.run_servo_command(angle)
            else:
                self.get_logger().warn(f"Unknown key pressed: '{key}'", throttle_duration_sec=1)
        
        # --- Cleanup on Exit ---
        self.get_logger().info("Ctrl+C detected. Shutting down and stopping robot.")
        if self.movement_thread is not None and self.movement_thread.is_alive():
            self.stop_event.set()
            self.movement_thread.join()
        self.twist_publisher.publish(Twist()) # Final safety stop
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTunerNode()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
