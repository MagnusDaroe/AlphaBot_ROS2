#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import RPi.GPIO as GPIO
import math

class AlphaBotNode(Node):
    last_j1_angle = 90
    last_j2_angle = 90
    
    def __init__(self):
        super().__init__('alphabot_node')

        # Initialize GPIO pins for motors
        self.IN1 = 12
        self.IN2 = 13
        self.IN3 = 20  
        self.IN4 = 21
        self.ENA = 6
        self.ENB = 26
        
        # Define servo pins
        self.S1 = 27
        self.S2 = 22

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.S1, GPIO.OUT)
        GPIO.setup(self.S2, GPIO.OUT)

        # Initialize PWM for motors
        self.PWMA = GPIO.PWM(self.ENA, 500)
        self.PWMB = GPIO.PWM(self.ENB, 500)
        self.PWMA.start(50)  # Default duty cycle: 50%
        self.PWMB.start(50)
        
        # Initialize PWM for servos
        self.J1 = GPIO.PWM(self.S1, 50)
        self.J2 = GPIO.PWM(self.S2, 50)
        self.J1.start(7.5) # Default angle: 90°
        self.J2.start(7.5) # Default angle: 90°

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
        # Extract linear and angular velocities from the Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        
        # Convert normalized linear and angular velocities to angles for the servos
        j1_angle, j2_angle = self.calculate_angle(angular_y, angular_x)
        
        # Set the angle of the servos
        if j1_angle != self.last_j1_angle:
            print(j1_angle)
            self.setAngle(self.J1, j1_angle)
            self.last_j1_angle = j1_angle
        if j2_angle != self.last_j2_angle:
            print(j2_angle)
            self.setAngle(self.J2, j2_angle)
            self.last_j2_angle = j2_angle

        # Calculate motor speeds based on linear and angular velocities
        left_speed, right_speed = self.calculate_speeds(linear_x, angular_z)   

        # Send the calculated speeds to the motors
        self.setSpeed(left_speed, right_speed)

    def publish_sensor_states(self):
        """Read sensor states and publish them."""
        dr_status = GPIO.input(self.DR)
        dl_status = GPIO.input(self.DL)

        # Publish as a single Int8 value (LSB = DL, MSB = DR)
        sensor_state = (dr_status << 1) | dl_status
        msg = Int8(data=sensor_state)
        self.ir_publisher.publish(msg)
        
    def calculate_speeds(self, linear_x, angular_z):
        """Calculate motor speeds based on linear and angular velocities."""
         # Normalize angular_z to [-1, 1] range
        angular_z = max(-1.0, min(1.0, angular_z))
        
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        # Ensure that the calculated speeds are within the range [-1.0, 1.0]
        max_magnitude = max(abs(left_speed), abs(right_speed), 1.0)
        left_speed /= max_magnitude
        right_speed /= max_magnitude

        # Scale speeds to an integer range suitable for your motors (assuming 100 is max PWM)
        left_speed = int(left_speed * 100)
        right_speed = int(right_speed * 100)
        
        return left_speed, right_speed

    def setSpeed(self, left_speed, right_speed):
        """
        Set the speed and direction of the motors using logical variables for GPIO control.
        Args:
            left_speed (int): Speed for the left motor (-100 to 100).
            right_speed (int): Speed for the right motor (-100 to 100).
        """
        # Ensure speeds are within the range [-100, 100], and reverse the direction left motor to match the same direction as the right motor
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(-right_speed, 100), -100)

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
    
    def calculate_angle(self, angular_x, angular_y):
        """Calculate the angle of the servos. This is based on a normalized linear_x and angular_y."""
        j1_angle = self.last_j1_angle + angular_x
        j2_angle = self.last_j2_angle + angular_y
        
        #Make sure it is within the range of 0-180
        if j1_angle >= 180:
            j1_angle = 180
        if j1_angle <= 0:
            j1_angle = 0
        if j2_angle >= 180:
            j2_angle = 180
        if j2_angle <= 0:
            j2_angle = 0
        
        return j1_angle, j2_angle
    
    def setAngle(self, servo, angle):
        """Set the angle of the servo."""
        duty = 2.5 + (angle / 180.0) * 10  # Map 0-180° to 2.5%-12.5%
        servo.ChangeDutyCycle(duty)

    def destroy_node(self):
        """Clean up GPIO on shutdown."""
        self.setSpeed(0, 0)
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
