

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer_period_ = 0.033  # 30 fps
        # self.timer_period_ = 0.066  # 15fps
        self.timer_ = self.create_timer(self.timer_period_, self.publish_image)
        # self.cap = cv2.VideoCapture('/home/jon/Downloads/orb_init_vid1.mp4') 
        self.cap = cv2.VideoCapture('/home/jon/Downloads/chairs1.mp4') 



        # self.timer_ = self.create_timer(0.5, self.publish_image)
        # self.image_index = 0
        # self.image_loop_forward = True
        self.bridge = CvBridge()

    def publish_image(self):
        ret, image = self.cap.read()
        if not ret:
            # If the frame was not read, the video has ended. Reset the video capture object.
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            _, image = self.cap.read()
        # if ret:
        #     msg = Image()
        #     msg.height = frame.shape[0]
        #     msg.width = frame.shape[1]
        #     msg.encoding = 'bgr8'
        #     msg.data = frame.tobytes()
        #     self.publisher_.publish(msg)
        # else:
        #     self.get_logger().warn("Failed to read frame from video file!")
        #     self.cap.release()
        #     self.timer_.cancel()

        # Load an image from file
        
        # image = cv2.imread(f'/home/jon/Downloads/sequence_22/images/0000{self.image_index}.jpg')  
        # if image is None:
        #     self.get_logger().error('Failed to load image.')
        #     return
        
        # if self.image_loop_forward:
        #     self.image_index += 1
        #     if self.image_index == 20:
        #         self.image_loop_forward = False
        # else:
        #     self.image_index -= 1
        #     if self.image_index == 0:
        #         self.image_loop_forward = True

        # # Convert the image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

        # Publish the image
        self.publisher_.publish(ros_image)
        # self.get_logger().info('Image published at {}'.format(time.time()))

def main(args=None):
    print('Hi from test_package.')
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()