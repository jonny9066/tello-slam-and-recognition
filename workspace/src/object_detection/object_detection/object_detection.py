import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

black_box = np.zeros((10,10))
cv2.imshow("frame",black_box)
from ultralytics import YOLO #must import after first imshow or freeze.

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt") # model parameters

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return
        results = self.model(cv_image)
        annotated = results[0].plot()
        cv2.imshow("frame",annotated)

        cv2.waitKey(1)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

        
# cap = cv2.VideoCapture(0)
# ret, frame = cap.read()
# cv2.imshow("frame",frame)

# from ultralytics import YOLO #must import after first imshow or freeze.
# model = YOLO("yolov8n.pt")


# while True:
#     ret, frame = cap.read()

#     results = model(frame)
#     # cv2.imshow("frame",frame)

#     annotated = results[0].plot()
#     cv2.imshow("frame",annotated)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

#         # Display the image or perform any desired processing
#         cv2.imshow("Image", cv_image)
#         cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()