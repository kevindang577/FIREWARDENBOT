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

class FastFireDetector : public rclcpp::Node
{
public:
    FastFireDetector() : Node("fast_fire_detector")
    {
        // Parameters
        this->declare_parameter("camera_topic", "/camera/image_raw");
        this->declare_parameter("confidence_threshold", 0.7);
        this->declare_parameter("min_fire_area", 500);
        this->declare_parameter("max_fire_area", 50000);
        
        // Get parameters
        camera_topic_ = this->get_parameter("camera_topic").as_string();
        confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
        min_fire_area_ = this->get_parameter("min_fire_area").as_int();
        max_fire_area_ = this->get_parameter("max_fire_area").as_int();
        
        // Subscribers
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10,
            std::bind(&FastFireDetector::camera_callback, this, std::placeholders::_1));
        
        // Publishers
        fire_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/fire_detection/detected", 10);
        fire_location_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/fire_detection/location", 10);
        fire_alert_pub_ = this->create_publisher<std_msgs::msg::String>("/fire_detection/alert", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/fire_detection/debug_image", 10);
        
        // Initialize fire detection parameters
        setup_fire_detection();
        
        RCLCPP_INFO(this->get_logger(), "Fast Fire Detector initialized");
    }

private:
    void setup_fire_detection()
    {
        // Define fire color ranges in HSV
        // Orange-red fire colors
        fire_ranges_.push_back(std::make_pair(cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255)));
        fire_ranges_.push_back(std::make_pair(cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255)));
        // Yellow flames
        fire_ranges_.push_back(std::make_pair(cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255)));
        
        // Morphological kernel
        morph_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    }
    
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // Detect fire
            auto detections = detect_fire(image);
            
            // Process detections
            if (!detections.empty()) {
                publish_fire_detection(detections, msg->header);
            } else {
                // Publish no fire detected
                std_msgs::msg::Bool no_fire_msg;
                no_fire_msg.data = false;
                fire_detected_pub_->publish(no_fire_msg);
            }
            
            // Publish debug image
            publish_debug_image(image, detections, msg->header);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    struct FireDetection {
        cv::Rect bbox;
        cv::Point2f center;
        double confidence;
        double area;
    };
    
    std::vector<FireDetection> detect_fire(const cv::Mat& image)
    {
        std::vector<FireDetection> detections;
        
        // Convert to HSV
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // Create combined fire mask
        cv::Mat fire_mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
        
        for (const auto& range : fire_ranges_) {
            cv::Mat color_mask;
            cv::inRange(hsv, range.first, range.second, color_mask);
            cv::bitwise_or(fire_mask, color_mask, fire_mask);
        }
        
        // Morphological operations
        cv::morphologyEx(fire_mask, fire_mask, cv::MORPH_OPEN, morph_kernel_);
        cv::morphologyEx(fire_mask, fire_mask, cv::MORPH_CLOSE, morph_kernel_);
        
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fire_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            
            if (area >= min_fire_area_ && area <= max_fire_area_) {
                // Calculate confidence based on area and shape
                double area_confidence = std::min(area / 10000.0, 1.0);
                
                // Calculate shape confidence (fires tend to be irregular)
                double perimeter = cv::arcLength(contour, true);
                double circularity = 4 * M_PI * area / (perimeter * perimeter);
                double shape_confidence = 1.0 - circularity; // Lower circularity = higher confidence
                
                double confidence = (area_confidence + shape_confidence) / 2.0;
                
                if (confidence >= confidence_threshold_) {
                    cv::Rect bbox = cv::boundingRect(contour);
                    cv::Point2f center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
                    
                    detections.push_back({bbox, center, confidence, area});
                }
            }
        }
        
        return detections;
    }
    
    void publish_fire_detection(const std::vector<FireDetection>& detections, 
                               const std_msgs::msg::Header& header)
    {
        // Find best detection (highest confidence)
        auto best_detection = std::max_element(detections.begin(), detections.end(),
            [](const FireDetection& a, const FireDetection& b) {
                return a.confidence < b.confidence;
            });
        
        // Publish fire detected flag
        std_msgs::msg::Bool fire_msg;
        fire_msg.data = true;
        fire_detected_pub_->publish(fire_msg);
        
        // Publish fire location
        geometry_msgs::msg::PointStamped location_msg;
        location_msg.header = header;
        location_msg.point.x = best_detection->center.x;
        location_msg.point.y = best_detection->center.y;
        location_msg.point.z = 0.0;
        fire_location_pub_->publish(location_msg);
        
        // Publish fire alert
        std_msgs::msg::String alert_msg;
        alert_msg.data = "FIRE DETECTED! Confidence: " + std::to_string(best_detection->confidence) +
                        ", Location: (" + std::to_string(best_detection->center.x) + ", " +
                        std::to_string(best_detection->center.y) + ")";
        fire_alert_pub_->publish(alert_msg);
        
        RCLCPP_WARN(this->get_logger(), "FIRE DETECTED! Confidence: %.2f, Area: %.0f",
                   best_detection->confidence, best_detection->area);
    }
    
    void publish_debug_image(const cv::Mat& image, const std::vector<FireDetection>& detections,
                           const std_msgs::msg::Header& header)
    {
        cv::Mat debug_image = image.clone();
        
        // Draw detections
        for (const auto& detection : detections) {
            // Draw bounding box
            cv::rectangle(debug_image, detection.bbox, cv::Scalar(0, 255, 0), 2);
            
            // Draw center point
            cv::circle(debug_image, detection.center, 5, cv::Scalar(0, 255, 0), -1);
            
            // Draw confidence text
            std::string label = "FIRE: " + std::to_string(detection.confidence).substr(0, 4);
            cv::putText(debug_image, label, 
                       cv::Point(detection.bbox.x, detection.bbox.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
        
        // Convert back to ROS message
        cv_bridge::CvImage debug_cv_image;
        debug_cv_image.header = header;
        debug_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        debug_cv_image.image = debug_image;
        
        debug_image_pub_->publish(*debug_cv_image.toImageMsg());
    }
    
    // Member variables
    std::string camera_topic_;
    double confidence_threshold_;
    int min_fire_area_;
    int max_fire_area_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fire_detected_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr fire_location_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fire_alert_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    
    std::vector<std::pair<cv::Scalar, cv::Scalar>> fire_ranges_;
    cv::Mat morph_kernel_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastFireDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
