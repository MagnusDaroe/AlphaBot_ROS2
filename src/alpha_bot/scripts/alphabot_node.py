import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import RPi.GPIO as GPIO

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
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Determine movement based on linear and angular velocity
        if linear_x > 0:
            self.forward()
        elif linear_x < 0:
            self.backward()
        elif angular_z > 0:
            self.left()
        elif angular_z < 0:
            self.right()
        else:
            self.stop()

    def publish_sensor_states(self):
        """Read sensor states and publish them."""
        dr_status = GPIO.input(self.DR)
        dl_status = GPIO.input(self.DL)

        # Publish as a single Int8 value (LSB = DL, MSB = DR)
        sensor_state = (dr_status << 1) | dl_status
        msg = Int8(data=sensor_state)
        self.ir_publisher.publish(msg)

    def forward(self):
        """Move the robot forward."""
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.get_logger().info('Moving forward.')

    def backward(self):
        """Move the robot backward."""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.get_logger().info('Moving backward.')

    def left(self):
        """Turn the robot left."""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.get_logger().info('Turning left.')

    def right(self):
        """Turn the robot right."""
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.get_logger().info('Turning right.')

    def stop(self):
        """Stop the robot."""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.get_logger().info('Stopping.')

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
