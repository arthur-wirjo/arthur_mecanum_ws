import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse
import time

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # --- Argument Parsing ---
    # This allows us to give commands from the terminal
    parser = argparse.ArgumentParser(description='Send a timed Twist command for PID tuning.')
    parser.add_argument('-x', '--linear-x', type=float, default=0.0, help='Linear velocity in x (m/s)')
    parser.add_argument('-z', '--angular-z', type=float, default=0.0, help='Angular velocity in z (rad/s)')
    parser.add_argument('-t', '--time', type=float, default=1.0, help='Duration to send the command (seconds)')
    
    # We need to tell argparse to ignore ROS-specific arguments
    parsed_args, _ = parser.parse_known_args()

    # --- Node and Publisher Setup ---
    node = rclpy.create_node('pid_tuner_node')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    # --- Create the Twist Message ---
    twist = Twist()
    twist.linear.x = parsed_args.linear_x
    twist.angular.z = parsed_args.angular_z

    # --- Main Logic ---
    node.get_logger().info(
        f"Sending command: linear.x={twist.linear.x:.2f} m/s, "
        f"angular.z={twist.angular.z:.2f} rad/s for {parsed_args.time:.2f} seconds."
    )

    # Get the start time
    start_time = time.time()
    
    # Loop for the specified duration, publishing the command continuously
    # This ensures the micro-ROS agent receives the command reliably
    while (time.time() - start_time) < parsed_args.time:
        publisher.publish(twist)
        time.sleep(0.05)  # Publish at ~20Hz

    # --- Send a Stop Command ---
    # This is crucial! After the test, we command the robot to stop.
    stop_twist = Twist()
    publisher.publish(stop_twist)
    node.get_logger().info("Time elapsed. Sending stop command.")

    # Give it a moment to publish the stop command before shutting down
    time.sleep(0.2)

    # --- Cleanup ---
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
