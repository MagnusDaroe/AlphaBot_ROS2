#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class AlphabotCam(Node):
    def __init__(self, topic_name="/alphabot/out/cam", frame_rate=30):
        """
        Initialize the node, create a publisher for the compressed video stream, and set up the Pi camera.

        Args:
            topic_name (str): The name of the topic to publish the compressed video stream.
            frame_rate (int): The frame rate for the video stream.
        """
        super().__init__("alphabot_cam")
        self.publisher = self.create_publisher(CompressedImage, topic_name, 10)
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
        Capture a frame from the Pi camera, compress it to JPEG format, convert it to a ROS CompressedImage message, and publish it.
        """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame from camera.")
            return

        try:
            # Compress the frame using JPEG encoding (you can change this to PNG if you prefer)
            ret, jpeg_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ret:
                self.get_logger().warning("Failed to encode image as JPEG.")
                return

            # Create the CompressedImage message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_image_msg.format = "jpeg"  # You can change this to "png" if using PNG compression
            compressed_image_msg.data = jpeg_frame.tobytes()  # The image data in bytes

            # Publish the compressed image message
            self.publisher.publish(compressed_image_msg)
        except Exception as e:
            self.get_logger().error(f"Error compressing frame: {e}")

    def destroy_node(self):
        """
        Release the camera resource and clean up when the node is destroyed.
        """
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AlphabotCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
