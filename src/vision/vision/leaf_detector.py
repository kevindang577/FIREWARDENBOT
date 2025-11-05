#!/usr/bin/env python3
"""
Leaf Detection and Color Classification Node for Fire Warden Bot
Processes camera feed to detect and categorize autumn leaves by color
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import json
from datetime import datetime

class LeafDetector(Node):
    def __init__(self):
        super().__init__('leaf_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/model/drone1/camera',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String,
            '/leaf_detections',
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/leaf_detector/annotated_image',
            10
        )
        
        self.leaf_location_pub = self.create_publisher(
            PointStamped,
            '/leaf_detector/leaf_locations',
            10
        )
        
        # Color ranges for different autumn leaf types (HSV)
        self.color_ranges = {
            'bright_orange': {
                'lower': np.array([5, 150, 150]),
                'upper': np.array([15, 255, 255]),
                'rgb_color': (255, 165, 0)  # For visualization
            },
            'red_orange': {
                'lower': np.array([0, 120, 120]),
                'upper': np.array([10, 255, 255]),
                'rgb_color': (255, 69, 0)
            },
            'yellow_orange': {
                'lower': np.array([15, 100, 100]),
                'upper': np.array([35, 255, 255]),
                'rgb_color': (255, 215, 0)
            },
            'brown': {
                'lower': np.array([8, 50, 20]),
                'upper': np.array([20, 150, 100]),
                'rgb_color': (139, 69, 19)
            },
            'dark_red': {
                'lower': np.array([170, 100, 50]),
                'upper': np.array([180, 255, 150]),
                'rgb_color': (139, 0, 0)
            }
        }
        
        # Detection parameters
        self.min_contour_area = 500  # Minimum area for leaf detection
        self.max_contour_area = 10000  # Maximum area to avoid false positives
        
        # Detection statistics
        self.detection_count = 0
        self.color_statistics = {color: 0 for color in self.color_ranges.keys()}
        
        self.get_logger().info('Leaf Detector Node initialized - Ready for foliage detection!')
        
    def image_callback(self, msg):
        """Process incoming camera images for leaf detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect and classify leaves
            detections, annotated_image = self.detect_leaves(cv_image)
            
            # Publish results
            if detections:
                self.publish_detections(detections, msg.header)
                
            # Publish annotated image
            self.publish_annotated_image(annotated_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_leaves(self, image):
        """Detect and classify leaves by color in the image"""
        detections = []
        annotated_image = image.copy()
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Apply Gaussian blur to reduce noise
        hsv_blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
        
        for color_name, color_info in self.color_ranges.items():
            # Create mask for this color range
            mask = cv2.inRange(hsv_blurred, color_info['lower'], color_info['upper'])
            
            # Apply morphological operations to clean up the mask
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by size
                if self.min_contour_area < area < self.max_contour_area:
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center point
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Create detection record
                    detection = {
                        'color': color_name,
                        'area': float(area),
                        'center': (center_x, center_y),
                        'bounding_box': (x, y, w, h),
                        'confidence': self.calculate_confidence(contour, mask, area)
                    }
                    
                    detections.append(detection)
                    
                    # Update statistics
                    self.color_statistics[color_name] += 1
                    
                    # Draw on annotated image
                    self.draw_detection(annotated_image, detection, color_info['rgb_color'])
        
        # Add detection summary to image
        self.add_detection_summary(annotated_image, detections)
        
        return detections, annotated_image
    
    def calculate_confidence(self, contour, mask, area):
        """Calculate detection confidence based on shape and color consistency"""
        # Shape analysis - leaves tend to be more elongated
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = max(w, h) / min(w, h)
        
        # Color consistency - check how much of the bounding box is the target color
        roi_mask = mask[y:y+h, x:x+w]
        color_ratio = np.sum(roi_mask > 0) / (w * h) if (w * h) > 0 else 0
        
        # Combine factors for confidence score
        shape_score = min(1.0, aspect_ratio / 3.0)  # Prefer some elongation
        color_score = color_ratio
        size_score = min(1.0, area / 5000.0)  # Prefer larger detections
        
        confidence = (shape_score * 0.3 + color_score * 0.5 + size_score * 0.2)
        return min(1.0, confidence)
    
    def draw_detection(self, image, detection, color):
        """Draw detection visualization on the image"""
        x, y, w, h = detection['bounding_box']
        center_x, center_y = detection['center']
        
        # Draw bounding rectangle
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        
        # Draw center point
        cv2.circle(image, (center_x, center_y), 5, color, -1)
        
        # Add label
        label = f"{detection['color']} ({detection['confidence']:.2f})"
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        cv2.rectangle(image, (x, y - label_size[1] - 10), 
                     (x + label_size[0], y), color, -1)
        cv2.putText(image, label, (x, y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def add_detection_summary(self, image, detections):
        """Add detection summary overlay to the image"""
        height, width = image.shape[:2]
        
        # Create semi-transparent overlay
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (300, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # Add text information
        y_offset = 30
        cv2.putText(image, f"Leaf Detections: {len(detections)}", 
                   (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        y_offset += 20
        cv2.putText(image, f"Total Found: {self.detection_count + len(detections)}", 
                   (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show color breakdown
        y_offset += 15
        for color, count in self.color_statistics.items():
            if count > 0:
                cv2.putText(image, f"{color}: {count}", 
                           (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                y_offset += 15
    
    def publish_detections(self, detections, header):
        """Publish leaf detection results"""
        # Update detection count
        self.detection_count += len(detections)
        
        # Create detection message
        detection_data = {
            'timestamp': datetime.now().isoformat(),
            'frame_id': header.frame_id,
            'total_detections': len(detections),
            'detections': []
        }
        
        for detection in detections:
            detection_data['detections'].append({
                'color_category': detection['color'],
                'area_pixels': detection['area'],
                'center_pixel': detection['center'],
                'confidence': detection['confidence'],
                'bounding_box': detection['bounding_box']
            })
            
            # Publish individual leaf location
            location_msg = PointStamped()
            location_msg.header = header
            location_msg.point.x = float(detection['center'][0])  # Pixel coordinates
            location_msg.point.y = float(detection['center'][1])
            location_msg.point.z = 0.0
            self.leaf_location_pub.publish(location_msg)
        
        # Publish complete detection data
        detection_msg = String()
        detection_msg.data = json.dumps(detection_data, indent=2)
        self.detection_pub.publish(detection_msg)
        
        # Log significant detections
        if len(detections) > 0:
            color_summary = {}
            for det in detections:
                color_summary[det['color']] = color_summary.get(det['color'], 0) + 1
            
            self.get_logger().info(f'Detected {len(detections)} leaves: {color_summary}')
    
    def publish_annotated_image(self, annotated_image, header):
        """Publish the annotated image with detection overlays"""
        try:
            # Convert back to ROS image message
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = header
            self.annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing annotated image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    leaf_detector = LeafDetector()
    
    try:
        rclpy.spin(leaf_detector)
    except KeyboardInterrupt:
        leaf_detector.get_logger().info('Leaf Detector shutting down...')
    finally:
        # Print final statistics
        total_detections = sum(leaf_detector.color_statistics.values())
        leaf_detector.get_logger().info(f'Final Detection Statistics:')
        leaf_detector.get_logger().info(f'Total leaves detected: {total_detections}')
        for color, count in leaf_detector.color_statistics.items():
            if count > 0:
                percentage = (count / total_detections) * 100 if total_detections > 0 else 0
                leaf_detector.get_logger().info(f'  {color}: {count} ({percentage:.1f}%)')
        
        leaf_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
