#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

#define lane 100000
#define corner 150000
#define T_Junction 170000
#define cross 280000

class Move {
 public:
  Move() {}

 private:
  void left() {}
  void right() {}
  void straight() {}
  void back() {}
};

class PIDController {
 public:
  PIDController(double kp, double kd)
      : kp_(kp), kd_(kd), previous_error_(0.0) {}

  double compute(double setpoint, double pv) {
    double error = setpoint - pv;
    double derivative = error - previous_error_;
    previous_error_ = error;
    return kp_ * error + kd_ * derivative;
  }

 private:
  double kp_;
  double kd_;
  double previous_error_;
};

class ImageCapture : public rclcpp::Node {
 public:
  ImageCapture() : Node("Image_capture"), pid_controller(0.01, 0.001) {
    subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ImageCapture::opencv_callback, this, _1));
    control_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        "diff_cont/cmd_vel_unstamped", 10);
    cv::namedWindow("Thinned Image", cv::WINDOW_NORMAL);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;
  PIDController pid_controller;
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat image, grey, binary_image, thinnedImage;
  // Determine the number of filtered contours
  size_t num_contours;
  geometry_msgs::msg::Twist control_msg;
  cv::Point2f centroid;
  int area = 0;

  void opencv_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      // Convert ROS image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image = cv_ptr->image;
      // cv::imshow("Image", image);

      // convert the BGR image to HSV colour space
      cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);

      // Apply binary thresholding
      cv::threshold(grey, binary_image, 127, 255, cv::THRESH_BINARY);
      cv::imshow("Thinned Image", binary_image);
      cv::findContours(binary_image, contours, cv::RETR_EXTERNAL,
                       cv::CHAIN_APPROX_SIMPLE);
      // Draw contours
      cv::Mat contours_image = cv::Mat::zeros(binary_image.size(), CV_8UC3);
      cv::drawContours(contours_image, contours, -1, cv::Scalar(0, 255, 0), 2);
      // cv::imshow("Thinned Image", contours_image);

      // // Apply Canny edge detector
      // cv::Mat edges_x,edges_y;
      // cv::GaussianBlur(binary_image,binary_image,cv::Size(5,5),1, 2);

      // cv::Sobel(binary_image,edges_x,CV_64F,2,0,5);
      // cv::convertScaleAbs(edges_x,edges_x);

      // cv::Sobel(binary_image,edges_y,CV_64F,0,2,5);
      // cv::convertScaleAbs(edges_y,edges_y);

      // std::vector<std::vector<cv::Point>> contours_x,contours_y;
      // cv::findContours(edges_x, contours_x, cv::RETR_EXTERNAL,
      // cv::CHAIN_APPROX_SIMPLE); cv::findContours(edges_y, contours_y,
      // cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // cv::Scalar contour_color = cv::Scalar(0,255,0); // Green color

      // cv::Mat contour_image_x = cv::Mat::zeros(edges_x.size(), CV_8UC3);
      // cv::Mat contour_image_y = cv::Mat::zeros(edges_y.size(), CV_8UC3);

      // for(int i=0 ; i < (int)contours_x.size(); i++)
      //     cv::drawContours(contour_image_x,contours_x,i,contour_color,2);
      // cv::imshow("Thinned Image", contour_image_x);
      // for(int i=0 ; i < (int)contours_y.size(); i++)
      //     cv::drawContours(contour_image_y,contours_y,i,contour_color,2);
      // cv::imshow("Fitted Lines", contour_image_y);

      cv::waitKey(1);

      // Process edges and drive the robot
      processEdges(contours);
    } catch (const cv_bridge::Exception &e) {
      // Handle the exception (e.g., log an error message)
      RCLCPP_ERROR(get_logger(),
                   "Error converting ROS image message to OpenCV image: %s",
                   e.what());
    }
  }

  void processEdges(const std::vector<std::vector<cv::Point>> &contours) {
    if (contours.size() == 1) {
      double area = cv::contourArea(contours[0]);
      cv::Moments moments = cv::moments(contours[0]);
      centroid.x = moments.m10 / moments.m00;
      centroid.y = moments.m01 / moments.m00;

      if (area < lane) {
        control_msg.linear.x = 0.00005 * (centroid.y); // Adjust linear velocity
        control_msg.angular.z =
            pid_controller.compute(image.cols / 2.0, centroid.x);
        RCLCPP_INFO(get_logger(), "Area of the lane : %2f", area);
      }

      else if (area < corner) {
        // control_msg.angular.z = (centroid.x - image.co   ls / 2.0) *
        // 0.01; control_msg.linear.x = 0.00005 * (centroid.y);
        // control_msg.angular.z = pid_controller.compute(image.cols / 2.0,
        // centroid.x);
        RCLCPP_INFO(get_logger(), "Area of the corner : %2f", area);
      }

      else if (area < T_Junction) {
        control_msg.linear.x = 0;
        control_msg.angular.z = 0;
        RCLCPP_INFO_ONCE(get_logger(), "Area of the TJunction : %2f", area);
      }
    }

    // control_msg.angular.z = 1;
    control_publisher_->publish(control_msg);

    // cv::ximgproc::thinning(mask, thinnedImage);

    // cv::imshow("Thinned Image", thinnedImage);

    // int height = thinnedImage.size().height;
    // int width = thinnedImage.size().width;

    // cv::Mat crop =
    // thinnedImage(cv::Range(1,height-1),cv::Range(1,width-1));

    // // Apply Hough transform to detect lines
    // std::vector<cv::Vec4i> lines;
    // cv::HoughLinesP(crop, lines, 3, CV_PI / 180, 100, 50, 100);

    // cv::Mat result = cv::Mat::zeros(image.size(), CV_8UC1);
    // cv::Mat contour_image = cv::Mat::zeros(image.size(), CV_8UC3);

    // if(lines.size()>0)
    // {
    //     // Fit a line to the detected points
    //     for (size_t i = 0; i < lines.size(); i++)
    //     {
    //         cv::Vec4i l = lines[i];
    //         cv::line(result, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
    //         cv::Scalar(255), 1, cv::LINE_AA);
    //     }

    //     cv::imshow("Fitted Lines", result);
    //     std::vector<std::vector<cv::Point>> contours;
    //     cv::findContours(result, contours, cv::RETR_EXTERNAL,
    //     cv::CHAIN_APPROX_SIMPLE); cv::Moments moments =
    //     cv::moments(contours[0]);

    //     cv::Scalar contour_color = cv::Scalar(0,255,0); // Green color
    //     for(int i=0 ; i < (int)contours.size(); i++)
    //         cv::drawContours(contour_image,contours,i,contour_color,2);

    //     // Show result
    //     if (lines.size() == 1) {

    //         centroid.x = moments.m10 / moments.m00;
    //         centroid.y = moments.m01 / moments.m00;

    //         control_msg.linear.x = 0.0007 * (centroid.y);  // Adjust linear
    //         velocity control_msg.angular.z = -(centroid.x - image.cols
    //         / 2.0)
    //         * 0.0008; RCLCPP_INFO(get_logger(), "Number of lines 1 : %ld",
    //         lines.size());

    //     } else {

    //         control_msg.linear.x = 0;  // Adjust linear velocity
    //         control_msg.angular.z = 0;   // Adjust angular velocity
    //         RCLCPP_INFO(get_logger(), "Number of lines > 1 : %ld",
    //         lines.size());

    //     }
    //     control_publisher_->publish(control_msg);
    // }
    // else {
    //     RCLCPP_INFO(get_logger(), "Number of lines 0 ");
    // }
    // // Example: Calculate the centroid of the edges
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageCapture>());
  rclcpp::shutdown();
  return 0;
}
