#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class AlphaBotControllerRemoteNode(Node):
    def __init__(self):
        super().__init__('alphabot_controller_remote_node')

        # Publisher for motor commands
        self.publisher = self.create_publisher(Twist, '/alphabot/in/motor_cmd', 10)

        # Initialize Pygame and Joystick
        pygame.init()
        pygame.joystick.init()

        # Check if a joystick is connected
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected.")
            exit(1)
        else:
            self.joystick = pygame.joystick.Joystick(0)  # Use the first joystick
            self.joystick.init()
            self.get_logger().info(f"Connected to joystick: {self.joystick.get_name()}")

        self.clock = pygame.time.Clock()
        self.running = True
        self.get_logger().info("AlphaBot controller remote control initialized.")

        # Flag to track when the stop command has been sent
        self.last_command_was_stop = False

    def run(self):
        """Main loop for reading controller inputs and sending motor commands."""
        while self.running:
            self.handle_events()
            self.clock.tick(60)

    def handle_events(self):
        """Handle Pygame events and send motor commands."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        # Get joystick input for the left joystick (axes 0 and 1)
        x_axis = self.joystick.get_axis(0)  # Left stick horizontal (right/left) for turning
        y_axis = -self.joystick.get_axis(1)  # Left stick vertical (up/down) for forward/backward

        # Reverse the x-axis
        x_axis = -x_axis

        # Deadzone for the left stick
        if abs(x_axis) < 0.1:
            x_axis = 0.0
        if abs(y_axis) < 0.1:
            y_axis = 0.0

        # Get joystick input for the right joystick (axes 2 and 3)
        angular_x = self.joystick.get_axis(4)  # Right stick horizontal (angular.x)
        angular_y = -self.joystick.get_axis(3)  # Right stick vertical (angular.y)

        # Deadzone for the right stick
        if abs(angular_x) < 0.3:
            angular_x = 0.0
        if abs(angular_y) < 0.3:
            angular_y = 0.0

        twist = self.calculate_twist(x_axis, y_axis, angular_x, angular_y)
        self.publisher.publish(twist)

    def calculate_twist(self, x_axis, y_axis, angular_x, angular_y):
        """Calculate Twist message from joystick input."""
        twist = Twist()

        # Left stick controls linear.x and angular.z
        twist.linear.x = -y_axis  # Negative because pushing up gives negative values (forward)
        twist.angular.z = x_axis  # Positive for right/clockwise, negative for left/counter-clockwise

        # Right stick controls angular.x and angular.y
        twist.angular.y = angular_y  # Horizontal movement of right stick
        twist.angular.x = angular_x  # Vertical movement of right stick

        self.get_logger().info(f"Publishing: linear.x={twist.linear.x}, angular.z={twist.angular.z}, angular.x={twist.angular.x}, angular.y={twist.angular.y}")
        return twist

    def destroy_node(self):
        """Clean up resources on shutdown."""
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    remote_node = AlphaBotControllerRemoteNode()
    try:
        remote_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        remote_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
