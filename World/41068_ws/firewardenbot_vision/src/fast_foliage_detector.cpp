#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <memory>

class FastFoliageDetector : public rclcpp::Node
{
public:
    FastFoliageDetector() : Node("fast_foliage_detector")
    {
        // Parameters
        this->declare_parameter("camera_topic", "/camera/image_raw");
        this->declare_parameter("confidence_threshold", 0.7);
        this->declare_parameter("min_foliage_area", 500);
        this->declare_parameter("max_foliage_area", 50000);
        
        // Get parameters
        camera_topic_ = this->get_parameter("camera_topic").as_string();
        confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
        min_foliage_area_ = this->get_parameter("min_foliage_area").as_int();
        max_foliage_area_ = this->get_parameter("max_foliage_area").as_int();
        
        // Subscribers
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10, 
            std::bind(&FastFoliageDetector::imageCallback, this, std::placeholders::_1));
        
        // Publishers
        foliage_alert_pub_ = this->create_publisher<std_msgs::msg::String>("/foliage_hazard/alert", 10);
        foliage_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/foliage_hazard/detected", 10);
        foliage_location_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/foliage_hazard/location", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/foliage_hazard/debug_image", 10);
        
        RCLCPP_INFO(this->get_logger(), "Fast Foliage Detector initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // Detect foliage hazards
            auto start_time = std::chrono::high_resolution_clock::now();
            std::vector<cv::Point> foliage_points = detectFoliageHazards(image);
            auto end_time = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            
            // Publish results
            if (!foliage_points.empty()) {
                publishFoliageDetection(foliage_points, msg->header);
                RCLCPP_DEBUG(this->get_logger(), "Detected %zu foliage hazard areas in %ld ms", 
                           foliage_points.size(), duration.count());
            }
            
            // Publish debug image if enabled
            publishDebugImage(image, foliage_points, msg->header);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    std::vector<cv::Point> detectFoliageHazards(cv::Mat& image)
    {
        std::vector<cv::Point> foliage_points;
        
        // Convert to HSV for better color analysis
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // Create masks for different foliage types
        cv::Mat dead_foliage_mask, dry_foliage_mask, dense_foliage_mask;
        
        // Dead foliage: brown/yellow colors
        cv::inRange(hsv, cv::Scalar(10, 50, 50), cv::Scalar(30, 255, 255), dead_foliage_mask);
        
        // Dry foliage: yellow-green to brown
        cv::inRange(hsv, cv::Scalar(25, 30, 30), cv::Scalar(60, 255, 200), dry_foliage_mask);
        
        // Dense green foliage (potential fuel load)
        cv::inRange(hsv, cv::Scalar(35, 40, 40), cv::Scalar(85, 255, 255), dense_foliage_mask);
        
        // Combine masks
        cv::Mat combined_mask;
        cv::bitwise_or(dead_foliage_mask, dry_foliage_mask, combined_mask);
        cv::bitwise_or(combined_mask, dense_foliage_mask, combined_mask);
        
        // Apply morphological operations to clean up
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN, kernel);
        
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // Filter contours by area and add center points
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area >= min_foliage_area_ && area <= max_foliage_area_) {
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 > 0) {
                    cv::Point center(moments.m10 / moments.m00, moments.m01 / moments.m00);
                    foliage_points.push_back(center);
                }
            }
        }
        
        return foliage_points;
    }
    
    void publishFoliageDetection(const std::vector<cv::Point>& foliage_points, const std_msgs::msg::Header& header)
    {
        // Publish detection flag
        std_msgs::msg::Bool detected_msg;
        detected_msg.data = true;
        foliage_detected_pub_->publish(detected_msg);
        
        // Publish alert message
        std_msgs::msg::String alert_msg;
        alert_msg.data = "Foliage hazard detected - " + std::to_string(foliage_points.size()) + " areas identified";
        foliage_alert_pub_->publish(alert_msg);
        
        // Publish locations
        for (const auto& point : foliage_points) {
            geometry_msgs::msg::PointStamped location_msg;
            location_msg.header = header;
            location_msg.point.x = point.x;
            location_msg.point.y = point.y;
            location_msg.point.z = 0.0;
            foliage_location_pub_->publish(location_msg);
        }
    }
    
    void publishDebugImage(cv::Mat& image, const std::vector<cv::Point>& foliage_points, const std_msgs::msg::Header& header)
    {
        // Draw detected foliage points
        for (const auto& point : foliage_points) {
            cv::circle(image, point, 10, cv::Scalar(0, 255, 255), 2);  // Yellow circles
            cv::putText(image, "FOLIAGE", cv::Point(point.x - 30, point.y - 15),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }
        
        // Convert back to ROS message and publish
        std_msgs::msg::Header debug_header = header;
        debug_header.frame_id = "foliage_detection_debug";
        
        cv_bridge::CvImage debug_bridge;
        debug_bridge.header = debug_header;
        debug_bridge.encoding = sensor_msgs::image_encodings::BGR8;
        debug_bridge.image = image;
        
        debug_image_pub_->publish(*debug_bridge.toImageMsg());
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr foliage_alert_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr foliage_detected_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr foliage_location_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    
    std::string camera_topic_;
    double confidence_threshold_;
    int min_foliage_area_;
    int max_foliage_area_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastFoliageDetector>());
    rclcpp::shutdown();
    return 0;
}
