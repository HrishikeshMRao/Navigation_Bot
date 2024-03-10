#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp> 
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class ImageCapture : public rclcpp::Node
{
public:

    ImageCapture() : Node("Image_capture")
    {
        subscriber_ = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&ImageCapture::opencv_callback, this, _1));
        control_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        //Initialize OpenCV window 
        cv::namedWindow("Image", cv::WINDOW_NORMAL);    
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;

    void opencv_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // Apply Canny edge detector
            cv::Mat edges;
            cv::Canny(cv_ptr->image, edges, 50, 150);  // Adjust threshold parameters as needed

            // Example: Display the edge-detected image
            cv::imshow("Image", edges);
            cv::waitKey(1);

            // Process edges and drive the robot
            processEdges(edges);
        }
        catch (const cv_bridge::Exception& e) {
            // Handle the exception (e.g., log an error message)
            RCLCPP_ERROR(get_logger(), "Error converting ROS image message to OpenCV image: %s", e.what());
        }
    }

    void processEdges(const cv::Mat& edges)
    {

        // Example: Calculate the centroid of the edges
        cv::Moments moments = cv::moments(edges);
        cv::Point2f centroid;
        centroid.x = moments.m10 / moments.m00;
        centroid.y = moments.m01 / moments.m00;

        // Example: Publish a Twist message for differential drive control
        geometry_msgs::msg::Twist control_msg;
        control_msg.linear.x = 0.007 * (3.0 * edges.rows / 4.0 - centroid.y);  // Adjust linear velocity
        control_msg.angular.z = (centroid.x - edges.cols / 2.0) * 0.01;   // Adjust angular velocity

        // Publish the control message
        control_publisher_->publish(control_msg);
    }
        
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageCapture>());
    rclcpp::shutdown();
    return 0;
}
