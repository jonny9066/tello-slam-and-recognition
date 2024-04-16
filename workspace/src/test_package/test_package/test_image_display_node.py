import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageReceiver(Node):
    def __init__(self):
        super().__init__('image_receiver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change 'image_topic' to your actual topic name
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Error converting ROS Image to OpenCV image: %s' % e)
            return

        # Display the received image
        cv2.imshow('Received Image', cv_image)
        cv2.waitKey(1)  # Needed to update the OpenCV window


def main(args=None):
    rclpy.init(args=args)

    image_receiver = ImageReceiver()

    rclpy.spin(image_receiver)

    # Destroy OpenCV windows upon shutdown
    cv2.destroyAllWindows()

    image_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()