#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/alphabot/out/ir',  # Topic you specified
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert the compressed image data into a numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)

            # Decode the image from the numpy array
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Show the image
            cv2.imshow("Compressed Camera Image", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error decoding the image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
