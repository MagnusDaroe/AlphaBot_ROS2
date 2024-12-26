#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class AlphaBotMouseRemoteNode(Node):
    def __init__(self):
        super().__init__('alphabot_mouse_remote_node')
        
        # Publisher for motor commands
        self.publisher = self.create_publisher(Twist, '/alphabot/in/motor_cmd', 10)

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 400))
        pygame.display.set_caption("AlphaBot Mouse Control")
        self.font = pygame.font.Font(None, 36)
        self.clock = pygame.time.Clock()
        self.running = True

        self.get_logger().info("AlphaBot mouse remote control initialized.")

        # Flag to track when the stop command has been sent
        self.last_command_was_stop = False

    def run(self):
        """Main loop for the Pygame interface."""
        while self.running:
            self.handle_events()
            self.update_screen()
            self.clock.tick(60)

    def handle_events(self):
        """Handle Pygame events and send motor commands."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        # Get mouse state
        mouse_buttons = pygame.mouse.get_pressed()
        if mouse_buttons[0]:  # Left mouse button is held down
            mouse_x, mouse_y = pygame.mouse.get_pos()
            twist = self.calculate_twist(mouse_x, mouse_y)
            self.publisher.publish(twist)
            self.last_command_was_stop = False
        else:
            # Publish stop command if it wasn't the last command
            if not self.last_command_was_stop:
                twist = Twist()  # Default values are zero
                self.publisher.publish(twist)
                self.get_logger().info("Published stop command.")
                self.last_command_was_stop = True

    def calculate_twist(self, mouse_x, mouse_y):
        """Calculate Twist message from mouse position."""
        width, height = self.screen.get_size()

        # Normalize mouse position to range [-1, 1]
        normalized_x = (mouse_x / width) * 2 - 1  # Range: [-1, 1]
        normalized_y = -((mouse_y / height) * 2 - 1)  # Range: [-1, 1], flipped for intuitive control

        twist = Twist()
        twist.linear.x = normalized_y  # Forward/backward speed
        twist.angular.z = normalized_x  # Turning speed

        self.get_logger().info(f"Publishing: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        return twist

    def update_screen(self):
        """Update the Pygame window."""
        self.screen.fill((0, 0, 0))  # Clear screen with black
        text = self.font.render("Hold mouse button to control", True, (255, 255, 255))
        self.screen.blit(text, (50, 180))
        pygame.display.flip()

    def destroy_node(self):
        """Clean up resources on shutdown."""
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    remote_node = AlphaBotMouseRemoteNode()
    try:
        remote_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        remote_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
