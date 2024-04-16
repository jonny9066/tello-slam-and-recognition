#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class WebcamPublisher : public rclcpp::Node {
public:
    WebcamPublisher() : Node("webcam_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&WebcamPublisher::capture_and_publish, this));
        cap_.open("http://10.0.0.8:4747/video"); // android ip webcam
        // cap_.open(0); // usb webcam
    }

private:
    void capture_and_publish() {
        cap_ >> frame;
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*img_msg);
        
    }
    cv::Mat frame;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebcamPublisher>());
    rclcpp::shutdown();
    return 0;
}