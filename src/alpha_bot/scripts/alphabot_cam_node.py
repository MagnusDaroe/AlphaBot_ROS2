import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PiCamStreamer(Node):
    def __init__(self, topic_name="pi_camera/image_raw", frame_rate=30):
        """
        Initialize the node, create a publisher for the video stream, and set up the Pi camera.

        Args:
            topic_name (str): The name of the topic to publish the video stream.
            frame_rate (int): The frame rate for the video stream.
        """
        super().__init__("pi_cam_streamer")
        self.publisher = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / frame_rate, self.publish_frame)

        # Initialize the camera
        self.cap = cv2.VideoCapture(0)  # Use the default camera (Pi camera is usually 0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera.")
            rclpy.shutdown()

        # Set camera parameters
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, frame_rate)

        self.get_logger().info(f"Pi Camera streamer initialized on topic '{topic_name}'.")

    def publish_frame(self):
        """
        Capture a frame from the Pi camera, convert it to a ROS Image message, and publish it.
        """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame from camera.")
            return

        try:
            # Convert the frame to a ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting frame to Image message: {e}")

    def destroy_node(self):
        """
        Release the camera resource and clean up when the node is destroyed.
        """
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PiCamStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
