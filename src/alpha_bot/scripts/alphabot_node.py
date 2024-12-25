#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import RPi.GPIO as GPIO
import math

class AlphaBotNode(Node):
    def __init__(self):
        super().__init__('alphabot_node')

        # Initialize GPIO pins for motors
        self.IN1 = 12
        self.IN2 = 13
        self.IN3 = 20  
        self.IN4 = 21
        self.ENA = 6
        self.ENB = 26

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)

        # Initialize PWM for motors
        self.PWMA = GPIO.PWM(self.ENA, 500)
        self.PWMB = GPIO.PWM(self.ENB, 500)
        self.PWMA.start(50)  # Default duty cycle: 50%
        self.PWMB.start(50)

        # Initialize GPIO pins for sensors
        self.DR = 16  # Right sensor
        self.DL = 19  # Left sensor
        GPIO.setup(self.DR, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.DL, GPIO.IN, GPIO.PUD_UP)

        # Subscribe to motor command topic
        self.motor_subscriber = self.create_subscription(
            Twist,
            '/alphabot/in/motor_cmd',
            self.motor_cmd_callback,
            10
        )

        # Publish sensor states
        self.ir_publisher = self.create_publisher(Int8, '/alphabot/out/ir', 10)
        self.timer = self.create_timer(0.1, self.publish_sensor_states)

        self.get_logger().info('AlphaBot node initialized.')

    def motor_cmd_callback(self, msg):
        """Handle motor commands."""
        # Extract linear and angular velocities from Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Normalize angular_z to [-1, 1] range
        angular_z = max(-1.0, min(1.0, angular_z))

        # Calculate motor speeds using a trigonometric model
        left_speed = linear_x * math.cos(angular_z * (math.pi / 4))  # Angular influence on left motor
        right_speed = linear_x * math.sin(angular_z * (math.pi / 4)) # Angular influence on right motor

        # Scale speeds to an integer range suitable for your motors
        left_speed = int(left_speed * 100)  # Assuming 100 is the max PWM duty cycle
        right_speed = int(right_speed * 100)

        # Send speeds to the motors
        self.setSpeed(left_speed, right_speed)

    def publish_sensor_states(self):
        """Read sensor states and publish them."""
        dr_status = GPIO.input(self.DR)
        dl_status = GPIO.input(self.DL)

        # Publish as a single Int8 value (LSB = DL, MSB = DR)
        sensor_state = (dr_status << 1) | dl_status
        msg = Int8(data=sensor_state)
        self.ir_publisher.publish(msg)

    def setSpeed(self, left_speed, right_speed):
        """
        Set the speed and direction of the motors using logical variables for GPIO control.
        Args:
            left_speed (int): Speed for the left motor (-100 to 100).
            right_speed (int): Speed for the right motor (-100 to 100).
        """
        # Ensure speeds are within the range [-100, 100]
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        # Initialize motor states
        IN1, IN2, IN3, IN4 = False, False, False, False

        # Determine the direction and state for the left motor
        if left_speed > 0:
            IN1, IN2 = True, False  # Forward
        elif left_speed < 0:
            IN1, IN2 = False, True  # Backward
 
        # Determine the direction and state for the right motor
        if right_speed > 0:
            IN3, IN4 = True, False  # Forward
        elif right_speed < 0:
            IN3, IN4 = False, True  # Backward

        # Apply motor states to GPIO
        GPIO.output(self.IN1, GPIO.HIGH if IN1 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH if IN2 else GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH if IN3 else GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH if IN4 else GPIO.LOW)

        # Set PWM duty cycles (absolute value of speed)
        self.PWMA.ChangeDutyCycle(abs(left_speed))
        self.PWMB.ChangeDutyCycle(abs(right_speed))

    def destroy_node(self):
        """Clean up GPIO on shutdown."""
        self.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AlphaBotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
