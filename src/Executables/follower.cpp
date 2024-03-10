#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp> 

using std::placeholders::_1;

class follower : public rclcpp::Node
{
public:
    follower(): Node("Motor_Driver_Node")
    {
        // Subscribe to the contour image topic
        contour_subscriber_ = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&follower::contourCallback, this, _1));

        // Advertise a Twist message to control the robot
        control_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/diff_cont/cmd_vel_unstamped", 10);

        //Initialize OpenCV window 
        cv::namedWindow("edge", cv::WINDOW_NORMAL);  
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr contour_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;
    std::vector<std::vector<cv::Point>> contours;
    cv_bridge::CvImagePtr cv_ptr;

    void contourCallback(const sensor_msgs::msg::Image::SharedPtr contour_msg)
    {
        // Convert sensor_msgs::Image to cv::Mat
        cv_ptr = cv_bridge::toCvCopy(contour_msg,  sensor_msgs::image_encodings::BGR8);

        cv::Mat image = cv_ptr->image;

        RCLCPP_INFO_ONCE(this->get_logger(),"Image Dimensions : (%d,%d)", image.rows,image.cols);
        RCLCPP_INFO_ONCE(this->get_logger(),"Image Encoding: %s", cv_ptr->encoding.c_str());
        cv::imshow("edge", image);

        // Find contours in the image
        // cv::findContours(cv_ptr->image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Process contours and find the midpoint between the two largest contours
        // if (contours.size() >= 2)
        // {
        //     // Sort contours by area (assumes larger area means closer to the robot)
        //     std::sort(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
        //         return cv::contourArea(a) > cv::contourArea(b);
        //     });

            // Get the centroids of the two largest contours
            // cv::Moments moments1 = cv::moments(contours[0]);
            // cv::Moments moments2 = cv::moments(contours[1]);

            // // Calculate the midpoints
            // cv::Point2f midpoint;
            // midpoint.x = (moments1.m10 / moments1.m00 + moments2.m10 / moments2.m00) / 2.0;
            // midpoint.y = (moments1.m01 / moments1.m00 + moments2.m01 / moments2.m00) / 2.0;

            // // Now you can use 'midpoint' as the calculated midpoint
            // // Example: Publish a Twist message for differential drive control
            // geometry_msgs::msg::Twist control_msg;
            // control_msg.linear.x = 0.5*(3*cv_ptr->image.rows/4-midpoint.y);  // Set linear velocity (adjust as needed)
            // control_msg.angular.z = (midpoint.x - cv_ptr->image.cols / 2.0) * 0.01;  // Adjust proportional control

            // control_publisher_->publish(control_msg);
        // }
        // else
        // RCLCPP_WARN(this->get_logger(), "Not enough contours found.");  
        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<follower>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}