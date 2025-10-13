#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

class WebcamPublisher : public rclcpp::Node {
 public:
  WebcamPublisher() : Node("webcam_publisher") {
    // Open the webcam (usually, 0 corresponds to the default camera)
    cap.open(0);

    // Create a publisher for image messages
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/webcam_image", 10);

    // Set up a timer to publish frames at a regular interval
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&WebcamPublisher::publishFrame, this));
  }

 private:
  cv::VideoCapture cap;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publishFrame() {
    // Read a frame from the webcam
    cv::Mat frame;
    cap >> frame;

    // Check if the frame is empty (end of video stream)
    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "End of video stream.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Frame dimensions: x=%d, y=%d, type=%d",
                frame.cols, frame.rows, frame.type());

    // Convert the OpenCV image to a ROS2 image message
    auto msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    RCLCPP_INFO(this->get_logger(), "Frame dimensions: msg=%d", msg->width);

    // // Publish the image message
    // publisher_->publish(*msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebcamPublisher>());
  rclcpp::shutdown();
  return 0;
}
