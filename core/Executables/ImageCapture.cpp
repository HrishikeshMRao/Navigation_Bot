// Copyright [2024] <Hrishikesh M Rao>
#include <functional>
#include <memory>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

// for quaternion → roll/pitch/yaw conversion
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Custom functions
#include "Astar.hpp" // Astar

#define Log(x, y) RCLCPP_INFO(get_logger(), x, y);

enum class State { SEARCH, ALIGN_CENTER, TURN, MOVE_FORWARD };
enum class Shape { LEFT, STRAIGHT, RIGHT };

struct PIDController {
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
  ImageCapture() : Node("Image_capture"), pid_controller(0.0007, 0.0001) {
    this->declare_parameter<std::string>("map", "1");
    // std::string map;
    // this->get_parameter("map", map);
    subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ImageCapture::opencv_callback, this, std::placeholders::_1));
    control_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        "diff_cont/cmd_vel_unstamped", 10);
    odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        "/diff_cont/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          GPSx = msg->pose.pose.position.x + 0.3;
          GPSy = msg->pose.pose.position.y - 0.74;
        });
    image_pub_ = image_transport::create_publisher(this, "camera/image");
    // Subscribe to IMU topic
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu_plugin/out", 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                            msg->orientation.z, msg->orientation.w);

          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

          // Convert yaw to degrees
          yaw_deg = yaw * 180.0 / M_PI;
        });

    std::string map = this->get_parameter("map").as_string();
    direction = Astar(map);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher image_pub_;
  PIDController pid_controller;
  State state;
  Shape shape;
  std::vector<int> direction;
  cv::Mat image, grey, binary_image, thinnedImage;
  geometry_msgs::msg::Twist control_msg;
  double yaw_deg, present_yaw, GPSx, GPSy;
  void opencv_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      for (int dir : direction) {
        RCLCPP_INFO(this->get_logger(), "direction: %d", dir);
      }
      // Convert ROS image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image = cv_ptr->image;

      // Convert the BGR image to grayscale
      cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);

      // Apply binary threshold to make it black & white
      cv::Mat binary_image;
      cv::threshold(grey, binary_image, 0, 255, CV_THRESH_BINARY);

      // Create a structuring element
      int morph_size = 2;
      cv::Mat element = getStructuringElement(
          cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
          cv::Point(morph_size, morph_size));
      cv::Mat open_morphed, close_morphed;

      // Opening
      cv::morphologyEx(binary_image, open_morphed, cv::MORPH_OPEN, element,
                       cv::Point(-1, -1), 1);

      // Closing
      cv::morphologyEx(open_morphed, close_morphed, cv::MORPH_CLOSE, element,
                       cv::Point(-1, -1), 1);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(close_morphed, contours, cv::RETR_EXTERNAL,
                       cv::CHAIN_APPROX_SIMPLE);
      cv::Moments m = cv::moments(contours[0]);
      cv::Point2f midpoint(float(m.m10 / m.m00), float(m.m01 / m.m00));

      double epsilon =
          0.005 * cv::arcLength(contours[0], true); // 2% of perimeter
      std::vector<cv::Point> approx;
      cv::approxPolyDP(contours[0], approx, epsilon, true);

      cv::Mat mask = cv::Mat::zeros(binary_image.size(), CV_8UC1);
      cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{approx},
                   cv::Scalar(255));

      cv::imshow("Mask", mask);
      // Detect corners using Shi–Tomasi corner detector
      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(
          mask, corners,
          4,   // maxCorners – increase for more detections
          0.4, // qualityLevel – smaller detects weaker corners
          170  // minDistance – minimum pixel spacing between corners
      );

      // Draw detected corners on a copy of the image
      cv::Mat corners_image;
      // cv::cvtColor(mask, corners_image, cv::COLOR_GRAY2BGR);
      image.copyTo(corners_image);
      cv::circle(corners_image, cv::Point(midpoint.x, midpoint.y), 5,
                 cv::Scalar(0, 0, 255), -1);
      for (const auto &pt : corners) {
        cv::circle(corners_image, pt, 4, cv::Scalar(0, 0, 255), -1);
        // Prepare the label text (index + coordinates)
        std::string label = " (" + std::to_string((int)pt.x) + "," +
                            std::to_string((int)pt.y) + ")";

        // Put the text near the corner
        cv::putText(corners_image, label, pt + cv::Point2f(5, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
      }
      // Draw all contours on the corners_image
      cv::drawContours(corners_image, contours, -1, cv::Scalar(0, 255, 0), 2);
      // Optionally draw image center for reference
      cv::circle(corners_image,
                 cv::Point(corners_image.cols / 2, corners_image.rows / 2), 5,
                 cv::Scalar(255, 0, 0), -1);

      std::string label =
          " (" + std::to_string(GPSx) + "," + std::to_string(GPSy) + ")";

      cv::putText(corners_image, label,
                  cv::Point(corners_image.cols / 2, corners_image.rows / 2) +
                      cv::Point(5, -5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

      // RCLCPP_INFO(this->get_logger(), "x(deg): %.2f", GPSx);
      // RCLCPP_INFO(this->get_logger(), "y(deg): %.2f", GPSy);

      // You can still compute an average X position for control if needed:
      switch (state) {
      case State::SEARCH: {
        control_msg.angular.z = 0;
        control_msg.linear.x =
            pid_controller.compute(image.rows, image.rows / 2.0);
        if (corners.size() > 1) {
          state = State::ALIGN_CENTER;
        }
        break;
      }

      case State::ALIGN_CENTER: {
        int maxX, maxY, minX, minY;
        maxX = std::max(corners[0].x, corners[1].x);
        minX = std::min(corners[0].x, corners[1].x);
        maxY = std::max(corners[0].y, corners[1].y);
        minY = std::min(corners[0].y, corners[1].y);
        int midX = (corners[0].x + corners[1].x) / 2.0f;
        int midY = (corners[0].y + corners[1].y) / 2.0f;

        cv::circle(corners_image, cv::Point(midX, midY), 5,
                   cv::Scalar(0, 255, 0), -1);

        control_msg.angular.z = 0;
        control_msg.linear.x = pid_controller.compute(image.rows / 2.0, midY);
        if (std::abs(image.rows / 2.0 - midY) < 0.001) {
          state = State::TURN;
          present_yaw = yaw_deg;
          if ((std::abs(midX - corners[0].x) > 5) &&
              (std::abs(midY - corners[0].y) > 5)) {
            int topX =
                corners[0].y > corners[1].y ? corners[0].x : corners[1].x;
            if (topX > midX) {
              shape = Shape::RIGHT;
            } else {
              shape = Shape::LEFT;
            }
          } else {
            if (direction[0] < 0)
              shape = Shape::LEFT;
            else if (direction[0] > 0)
              shape = Shape::RIGHT;
            else
              shape = Shape::STRAIGHT;
            RCLCPP_INFO(this->get_logger(), "direction(deg): %d",
                        static_cast<int>(shape));
            direction.erase(direction.begin());
          }
        }
        break;
      }

      case State::TURN: {
        int midX = (corners[0].x + corners[1].x) / 2.0f;
        int midY = (corners[0].y + corners[1].y) / 2.0f;

        cv::circle(corners_image, cv::Point(midX, midY), 5,
                   cv::Scalar(0, 255, 0), -1);
        auto normalize_angle = [](double angle) {
          while (angle > 180.0)
            angle -= 360.0;
          while (angle < -180.0)
            angle += 360.0;
          return angle;
        };
        switch (shape) {

        case Shape::LEFT: {
          control_msg.linear.x = 0.00;
          control_msg.angular.z =
              pid_controller.compute(normalize_angle(present_yaw + 90) * 30,
                                     normalize_angle(yaw_deg) * 30);
          // RCLCPP_INFO(this->get_logger(), "yaw_deg (deg): %.2f", yaw_deg);
          double target_yaw = normalize_angle(present_yaw + 90.0);
          double error = normalize_angle(yaw_deg - target_yaw);

          if (std::abs(error) < 0.001)
            state = State::MOVE_FORWARD;
          break;
        }
        case Shape::STRAIGHT: {
          state = State::MOVE_FORWARD;
          break;
        }
        case Shape::RIGHT: {
          control_msg.linear.x = 0.00;
          control_msg.angular.z =
              pid_controller.compute(normalize_angle(present_yaw - 90) * 30,
                                     normalize_angle(yaw_deg) * 30);
          // RCLCPP_INFO(this->get_logger(), "desired_yaw, yaw(deg):%.2f, %.2f",
          //             present_yaw - 90, yaw_deg);
          double target_yaw = normalize_angle(present_yaw - 90.0);
          double error = normalize_angle(yaw_deg - target_yaw);

          if (std::abs(error) < 0.001)
            state = State::MOVE_FORWARD;
          break;
        }
        }

        break;
      }

      case State::MOVE_FORWARD: {
        control_msg.angular.z = 0;
        control_msg.linear.x =
            pid_controller.compute(image.rows, image.rows / 2.0);
        if (corners.size() == 0) {
          state = State::SEARCH;
        }
        break;
      }
      }
      // publish image & control
      sensor_msgs::msg::Image::SharedPtr msg_out =
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", corners_image)
              .toImageMsg();
      image_pub_.publish(msg_out);

      control_publisher_->publish(control_msg);
    }

    catch (const cv_bridge::Exception &e) {
      // Handle the exception (e.g., log an error message)
      RCLCPP_ERROR(get_logger(),
                   "Error converting ROS image message to OpenCV image: %s",
                   e.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageCapture>());
  rclcpp::shutdown();
  return 0;
}
