#!/usr/bin/env python3
"""
Foliage Hazard Detection Node for Fire Warden Bot
Detects living and dead foliage that could become fire hazards
Uses multiple detection methods: color-based analysis, texture analysis, and machine learning
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from typing import List, Tuple, Dict, Optional

class FoliageHazardNode(Node):
    def __init__(self):
        super().__init__('foliage_hazard_node')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('thermal_topic', '/thermal_camera/image_raw')
        self.declare_parameter('detection_method', 'combined')  # 'color', 'thermal', 'texture', 'ml', 'combined'
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('publish_debug_images', True)
        self.declare_parameter('roi_enabled', True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        thermal_topic = self.get_parameter('thermal_topic').get_parameter_value().string_value
        
        self.camera_sub = self.create_subscription(
            Image, camera_topic, self.camera_callback, 10)
        self.thermal_sub = self.create_subscription(
            Image, thermal_topic, self.thermal_callback, 10)
        
        # Publishers
        self.foliage_alert_pub = self.create_publisher(String, '/foliage_hazard/alert', 10)
        self.foliage_location_pub = self.create_publisher(PointStamped, '/foliage_hazard/location', 10)
        self.foliage_confidence_pub = self.create_publisher(Float32, '/foliage_hazard/confidence', 10)
        self.debug_image_pub = self.create_publisher(Image, '/foliage_hazard/debug_image', 10)
        self.foliage_detected_pub = self.create_publisher(Bool, '/foliage_hazard/detected', 10)
        
        # Detection parameters
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.publish_debug = self.get_parameter('publish_debug_images').get_parameter_value().bool_value
        self.detection_method = self.get_parameter('detection_method').get_parameter_value().string_value
        self.roi_enabled = self.get_parameter('roi_enabled').get_parameter_value().bool_value
        
        # Foliage detection state
        self.last_detection_time = self.get_clock().now()
        self.detection_history = []
        self.thermal_image = None
        self.rgb_image = None
        
        # Color ranges for foliage detection (HSV)
        self.foliage_color_ranges = [
            # Dead/dry foliage (brown, yellow, orange)
            (np.array([8, 50, 20]), np.array([25, 255, 200])),
            # Yellowing foliage
            (np.array([25, 100, 100]), np.array([35, 255, 255])),
            # Living green foliage
            (np.array([35, 40, 40]), np.array([85, 255, 255]))
        ]
        
        # Texture analysis for dead foliage detection
        self.texture_detector = self.create_texture_detector()
        
        # Initialize deep learning model (if available)
        self.ml_detector = None
        try:
            self.ml_detector = self.create_ml_detector()
            self.get_logger().info("ML foliage detection model loaded successfully")
        except Exception as e:
            self.get_logger().warn(f"Could not load ML model: {e}")
        
        self.get_logger().info(f"Foliage Hazard Detection Node initialized with method: {self.detection_method}")
    
    def camera_callback(self, msg: Image):
        """Process RGB camera image for foliage hazard detection"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_foliage_detection()
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")
    
    def thermal_callback(self, msg: Image):
        """Process thermal camera image for foliage hazard detection"""
        try:
            self.thermal_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            if self.detection_method in ['thermal', 'combined']:
                self.process_foliage_detection()
        except Exception as e:
            self.get_logger().error(f"Error processing thermal image: {e}")
    
    def process_foliage_detection(self):
        """Main foliage hazard detection processing pipeline"""
        if self.rgb_image is None:
            return
        
        detections = []
        debug_image = self.rgb_image.copy()
        
        # Apply ROI if enabled
        roi_mask = self.get_roi_mask(self.rgb_image.shape) if self.roi_enabled else None
        
        # Color-based detection for different foliage types
        if self.detection_method in ['color', 'combined']:
            color_detections = self.detect_foliage_by_color(self.rgb_image, roi_mask)
            detections.extend(color_detections)
            debug_image = self.draw_detections(debug_image, color_detections, (0, 255, 0), "COLOR")
        
        # Thermal-based detection for dry vegetation
        if self.detection_method in ['thermal', 'combined'] and self.thermal_image is not None:
            thermal_detections = self.detect_foliage_by_thermal(self.thermal_image, roi_mask)
            detections.extend(thermal_detections)
            debug_image = self.draw_detections(debug_image, thermal_detections, (0, 0, 255), "THERMAL")
        
        # Texture analysis for dead foliage
        if self.detection_method in ['texture', 'combined']:
            texture_detections = self.detect_dead_foliage_texture(self.rgb_image, roi_mask)
            detections.extend(texture_detections)
            debug_image = self.draw_detections(debug_image, texture_detections, (255, 0, 0), "TEXTURE")
        
        # ML-based detection
        if self.detection_method in ['ml', 'combined'] and self.ml_detector is not None:
            ml_detections = self.detect_foliage_ml(self.rgb_image, roi_mask)
            detections.extend(ml_detections)
            debug_image = self.draw_detections(debug_image, ml_detections, (255, 255, 0), "ML")
        
        # Process combined detections
        final_detection = self.combine_detections(detections)
        
        if final_detection:
            self.publish_foliage_alert(final_detection, debug_image)
        else:
            # Publish no fire detected
            self.fire_detected_pub.publish(Bool(data=False))
        
        # Publish debug image
        if self.publish_debug:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
    
    def detect_fire_by_color(self, image: np.ndarray, roi_mask: Optional[np.ndarray] = None) -> List[Dict]:
        """Detect fire using color-based method"""
        detections = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Apply ROI mask if provided
        if roi_mask is not None:
            hsv = cv2.bitwise_and(hsv, hsv, mask=roi_mask)
        
        fire_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        
        # Combine all fire color ranges
        for lower, upper in self.fire_color_ranges:
            color_mask = cv2.inRange(hsv, lower, upper)
            fire_mask = cv2.bitwise_or(fire_mask, color_mask)
        
        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_OPEN, kernel)
        fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum fire area threshold
                # Calculate detection confidence based on area and color intensity
                confidence = min(area / 10000.0, 1.0)  # Normalize area to confidence
                
                if confidence > self.confidence_threshold:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x, center_y = x + w//2, y + h//2
                    
                    detections.append({
                        'method': 'color',
                        'confidence': confidence,
                        'bbox': (x, y, w, h),
                        'center': (center_x, center_y),
                        'area': area
                    })
        
        return detections
    
    def detect_fire_by_thermal(self, thermal_image: np.ndarray, roi_mask: Optional[np.ndarray] = None) -> List[Dict]:
        """Detect fire using thermal imaging"""
        detections = []
        
        # Apply ROI mask if provided
        if roi_mask is not None:
            thermal_image = cv2.bitwise_and(thermal_image, thermal_image, mask=roi_mask)
        
        # Threshold for high temperature (assuming normalized thermal image)
        _, hot_mask = cv2.threshold(thermal_image, 200, 255, cv2.THRESH_BINARY)
        
        # Morphological operations
        kernel = np.ones((7, 7), np.uint8)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, kernel)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(hot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:  # Minimum thermal area threshold
                # Calculate average temperature in the region
                mask = np.zeros(thermal_image.shape, dtype=np.uint8)
                cv2.fillPoly(mask, [contour], 255)
                avg_temp = cv2.mean(thermal_image, mask)[0]
                
                # Confidence based on temperature and area
                temp_confidence = (avg_temp - 150) / 105.0  # Normalize temperature
                area_confidence = min(area / 5000.0, 1.0)
                confidence = (temp_confidence + area_confidence) / 2.0
                
                if confidence > self.confidence_threshold:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x, center_y = x + w//2, y + h//2
                    
                    detections.append({
                        'method': 'thermal',
                        'confidence': confidence,
                        'bbox': (x, y, w, h),
                        'center': (center_x, center_y),
                        'area': area,
                        'temperature': avg_temp
                    })
        
        return detections
    
    def get_roi_mask(self, image_shape: Tuple[int, int, int]) -> np.ndarray:
        """Create ROI mask to focus on ground level areas"""
        mask = np.zeros(image_shape[:2], dtype=np.uint8)
        h, w = image_shape[:2]
        
        # Focus on lower 2/3 of image (ground level)
        mask[h//3:, :] = 255
        
        return mask
    
    def combine_detections(self, detections: List[Dict]) -> Optional[Dict]:
        """Combine multiple detection methods into final result"""
        if not detections:
            return None
        
        # Group nearby detections
        grouped_detections = self.group_nearby_detections(detections)
        
        # Find best detection group
        best_group = max(grouped_detections, key=lambda g: g['combined_confidence'])
        
        if best_group['combined_confidence'] > self.confidence_threshold:
            return best_group
        
        return None
    
    def group_nearby_detections(self, detections: List[Dict], distance_threshold: int = 100) -> List[Dict]:
        """Group detections that are close to each other"""
        groups = []
        used_detections = set()
        
        for i, detection in enumerate(detections):
            if i in used_detections:
                continue
            
            group = [detection]
            used_detections.add(i)
            
            for j, other_detection in enumerate(detections[i+1:], start=i+1):
                if j in used_detections:
                    continue
                
                # Calculate distance between detection centers
                dx = detection['center'][0] - other_detection['center'][0]
                dy = detection['center'][1] - other_detection['center'][1]
                distance = np.sqrt(dx*dx + dy*dy)
                
                if distance < distance_threshold:
                    group.append(other_detection)
                    used_detections.add(j)
            
            # Calculate group properties
            group_confidence = np.mean([d['confidence'] for d in group])
            group_center = (
                int(np.mean([d['center'][0] for d in group])),
                int(np.mean([d['center'][1] for d in group]))
            )
            
            # Boost confidence if multiple methods agree
            method_bonus = len(set(d['method'] for d in group)) * 0.1
            group_confidence = min(1.0, group_confidence + method_bonus)
            
            groups.append({
                'detections': group,
                'combined_confidence': group_confidence,
                'center': group_center,
                'methods': [d['method'] for d in group]
            })
        
        return groups
    
    def draw_detections(self, image: np.ndarray, detections: List[Dict], color: Tuple[int, int, int], method: str) -> np.ndarray:
        """Draw detection results on debug image"""
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            
            # Draw bounding box
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            
            # Draw confidence and method
            label = f"{method}: {confidence:.2f}"
            cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw center point
            center = detection['center']
            cv2.circle(image, center, 5, color, -1)
        
        return image
    
    def publish_foliage_alert(self, detection: Dict, debug_image: np.ndarray):
        """Publish fire detection alert and location"""
        current_time = self.get_clock().now()
        
        # Create alert message
        alert_data = {
            'timestamp': current_time.to_msg(),
            'confidence': detection['combined_confidence'],
            'methods': detection['methods'],
            'location': detection['center']
        }
        
        alert_msg = String()
        alert_msg.data = json.dumps(alert_data, default=str)
        self.fire_alert_pub.publish(alert_msg)
        
        # Publish location
        location_msg = PointStamped()
        location_msg.header.stamp = current_time.to_msg()
        location_msg.header.frame_id = "camera_frame"
        location_msg.point.x = float(detection['center'][0])
        location_msg.point.y = float(detection['center'][1])
        location_msg.point.z = 0.0
        self.fire_location_pub.publish(location_msg)
        
        # Publish confidence
        confidence_msg = Float32()
        confidence_msg.data = detection['combined_confidence']
        self.fire_confidence_pub.publish(confidence_msg)
        
        # Publish detection flag
        self.fire_detected_pub.publish(Bool(data=True))
        
        self.get_logger().warn(
            f"FIRE DETECTED! Confidence: {detection['combined_confidence']:.2f}, "
            f"Methods: {detection['methods']}, Location: {detection['center']}"
        )


class SmokeDetector:
    """Smoke detection using texture and motion analysis"""
    
    def __init__(self):
        self.prev_frame = None
        self.background_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=True)
    
    def detect_smoke(self, image: np.ndarray, roi_mask: Optional[np.ndarray] = None) -> List[Dict]:
        """Detect smoke using motion and texture analysis"""
        detections = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply ROI mask if provided
        if roi_mask is not None:
            gray = cv2.bitwise_and(gray, gray, mask=roi_mask)
        
        # Background subtraction for motion detection
        fg_mask = self.background_subtractor.apply(gray)
        
        # Texture analysis using LBP-like features
        texture_mask = self.analyze_texture(gray)
        
        # Combine motion and texture
        smoke_mask = cv2.bitwise_and(fg_mask, texture_mask)
        
        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        smoke_mask = cv2.morphologyEx(smoke_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(smoke_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Minimum smoke area
                # Calculate detection confidence
                confidence = min(area / 20000.0, 0.8)  # Lower confidence for smoke
                
                if confidence > 0.3:  # Lower threshold for smoke
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x, center_y = x + w//2, y + h//2
                    
                    detections.append({
                        'method': 'smoke',
                        'confidence': confidence,
                        'bbox': (x, y, w, h),
                        'center': (center_x, center_y),
                        'area': area
                    })
        
        return detections
    
    def analyze_texture(self, gray_image: np.ndarray) -> np.ndarray:
        """Analyze texture patterns typical of smoke"""
        # Calculate gradient magnitude
        grad_x = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3)
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        
        # Normalize and threshold
        grad_mag = cv2.normalize(grad_mag, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        _, texture_mask = cv2.threshold(grad_mag, 50, 255, cv2.THRESH_BINARY_INV)
        
        return texture_mask


class MLFireDetector:
    """Machine learning-based fire detection (placeholder for actual ML model)"""
    
    def __init__(self):
        # In a real implementation, you would load a trained model here
        # For example, using PyTorch or TensorFlow
        self.model_loaded = False
        
        # Placeholder for model initialization
        try:
            # self.model = torch.load('fire_detection_model.pth')
            # self.model.eval()
            self.model_loaded = True
        except:
            self.model_loaded = False
    
    def detect_foliage_ml(self, image: np.ndarray, roi_mask: Optional[np.ndarray] = None) -> List[Dict]:
        """Detect foliage hazards using machine learning model"""
        detections = []
        
        if not self.model_loaded:
            return detections
        
        # Placeholder implementation
        # In a real implementation, you would:
        # 1. Preprocess the image for your foliage detection model
        # 2. Run inference to classify foliage health/fire risk
        # 3. Post-process results
        # 4. Return detections in the expected format
        
        # For demonstration, return empty list
        return detections
    
    def create_texture_detector(self):
        """Create texture detector for dead foliage"""
        return self
    
    def create_ml_detector(self):
        """Create ML detector for foliage hazards"""
        return self
    
    def detect_dead_foliage_texture(self, image: np.ndarray, roi_mask: Optional[np.ndarray] = None) -> List[Dict]:
        """Detect dead foliage using texture analysis"""
        detections = []
        
        # Convert to grayscale for texture analysis
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply ROI mask if provided
        if roi_mask is not None:
            gray = cv2.bitwise_and(gray, gray, mask=roi_mask)
        
        # Calculate LBP (Local Binary Pattern) for texture
        # This is a simplified implementation
        lbp = self.calculate_lbp(gray)
        
        # Find areas with texture patterns typical of dead vegetation
        # Dead foliage typically has irregular, coarse textures
        texture_mask = cv2.threshold(lbp, 50, 255, cv2.THRESH_BINARY)[1]
        
        # Find contours
        contours, _ = cv2.findContours(texture_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                center_x, center_y = x + w//2, y + h//2
                
                # Calculate confidence based on texture variance
                roi = gray[y:y+h, x:x+w]
                variance = np.var(roi)
                confidence = min(variance / 1000.0, 1.0)
                
                if confidence > self.confidence_threshold:
                    detections.append({
                        'method': 'texture',
                        'confidence': confidence,
                        'bbox': (x, y, w, h),
                        'center': (center_x, center_y),
                        'area': area
                    })
        
        return detections
    
    def calculate_lbp(self, image: np.ndarray) -> np.ndarray:
        """Calculate Local Binary Pattern for texture analysis"""
        height, width = image.shape
        lbp = np.zeros((height-2, width-2), dtype=np.uint8)
        
        for i in range(1, height-1):
            for j in range(1, width-1):
                center = image[i, j]
                binary_string = ''
                
                # Check 8 neighbors
                neighbors = [
                    image[i-1, j-1], image[i-1, j], image[i-1, j+1],
                    image[i, j+1], image[i+1, j+1], image[i+1, j],
                    image[i+1, j-1], image[i, j-1]
                ]
                
                for neighbor in neighbors:
                    binary_string += '1' if neighbor >= center else '0'
                
                lbp[i-1, j-1] = int(binary_string, 2)
        
        return lbp


def main(args=None):
    rclpy.init(args=args)
    node = FoliageHazardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
