import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
# from tello_msg.msg import ObjectDetectionBox
from sensor_msgs.msg import CameraInfo

black_box = np.zeros((10,10))
cv2.imshow("frame",black_box)
cv2.waitKey(1)
from ultralytics import YOLO #must import after first cv2.imshow() or freeze.

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bounding_box_publisher = self.create_publisher(CameraInfo, 'object_detection_box', 10) 
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt") # model parameters

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return
        
        im_x_mid = cv_image.shape[1]//2
        im_y_mid = cv_image.shape[0]//2
        # print(im_x)
        # print(im_y)
        # exit()
        
        results = self.model(cv_image, verbose=False) #model returns detections for each image, here we have one image only
        if(len(results)==0):
            return
        
        # get all results
        boxes = results[0].boxes.xywh.tolist()
        classes = results[0].boxes.cls.tolist()
        names = results[0].names
        confidences = results[0].boxes.conf.tolist()
  

        # get coordinates first detected chair class
        # found_chair = False
        # coord_list = []
        min_box = "nope"
        min_dist = 100000
        for i, b, cls, conf in zip(range(len(boxes)), boxes, classes,confidences):
            if names[int(cls)] == 'chair' and conf > .72:
                name = names[int(cls)]
                box = b
                x,y,w,h = tuple(int(a) for a in box)
                dist_from_mid = abs(x-im_x_mid) + abs(y-im_y_mid)
                if dist_from_mid < min_dist: # bigger y is lower in the image
                    min_dist = dist_from_mid
                    min_box = (x,y,w,h)

                
                # coord_list.append(x,y,w,h)
                # found_chair = True
                # break
            
        # if not found_chair:
        #     return
            
        if min_box == "nope":
            return
        
        # min_index = min(range(len(coord_list)), key=lambda i: coord_list[i][:2]))

        # x,y,w,h = tuple(int(a) for a in box)

        x,y,w,h = min_box

        # Draw the rectangle
        # cv2.rectangle(cv_image, (x+int(w/2), y+int(h/2)), (x - int(w/2), y - int(h/2)), (0, 255, 0), 2)
        # cv2.imshow("frame",cv_image)
        # cv2.waitKey(1)
        
        # using a hack to avoid creating a custom message, which would require building ros1 bridge in a special way
        box_msg = CameraInfo()
        # std_msgs/Header header
        # uint32 height
        # uint32 width
        # string distortion_model
        # float64[] D
        # float64[9] K
        # float64[9] R
        # float64[12] P
        # uint32 binning_x
        # uint32 binning_y
        # sensor_msgs/RegionOfInterest roi
        box_msg.binning_x = x
        box_msg.binning_y = y
        box_msg.width = w
        box_msg.height = h
        box_msg.distortion_model = name
        self.bounding_box_publisher.publish(box_msg)

        
        # uncomment to show result image with all detections. For manually drawn rectange see above
        results = self.model(cv_image)
        annotated = results[0].plot()
        cv2.rectangle(annotated, (x+int(w/2), y+int(h/2)), (x - int(w/2), y - int(h/2)), (0, 255, 0), 5)


        im_x_mid = cv_image.shape[0]//2
        im_y_mid = cv_image.shape[1]//2

        # cv2.circle(annotated, (im_y_mid, im_x_mid), 5, (0, 255, 0), -1)  # Green circle
        # cv2.circle(annotated, (x, y), 5, (0, 255, 0), -1)  # Green circle
        # cv2.imshow("frame",annotated)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()
    
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()