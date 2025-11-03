#!/usr/bin/env python3
"""
Object Detection and Tracking Node for Fire Warden Bot
Detects and tracks objects like people, animals, vehicles in forest environments
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from typing import List, Dict, Tuple

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('publish_debug_images', True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load YOLO model for object detection
        self.load_yolo_model()
        
        # Initialize object tracker
        self.trackers = {}
        self.next_tracker_id = 0
        self.enable_tracking = self.get_parameter('enable_tracking').get_parameter_value().bool_value
        
        # Subscribers
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.camera_sub = self.create_subscription(
            Image, camera_topic, self.camera_callback, 10)
        
        # Publishers
        self.detections_pub = self.create_publisher(Detection2DArray, '/object_detection/detections', 10)
        self.alerts_pub = self.create_publisher(String, '/object_detection/alerts', 10)
        self.debug_image_pub = self.create_publisher(Image, '/object_detection/debug_image', 10)
        
        # Detection parameters
        self.confidence_threshold = self.get_parameter('detection_confidence').get_parameter_value().double_value
        self.nms_threshold = self.get_parameter('nms_threshold').get_parameter_value().double_value
        self.publish_debug = self.get_parameter('publish_debug_images').get_parameter_value().bool_value
        
        # Classes of interest for forest environment
        self.classes_of_interest = {
            0: 'person',
            1: 'bicycle', 
            2: 'car',
            3: 'motorcycle',
            5: 'bus',
            7: 'truck',
            14: 'bird',
            15: 'cat',
            16: 'dog',
            17: 'horse',
            18: 'sheep',
            19: 'cow',
            20: 'elephant',
            21: 'bear',
            22: 'zebra',
            23: 'giraffe'
        }
        
        self.get_logger().info("Object Detection Node initialized")
    
    def load_yolo_model(self):
        """Load YOLO model for object detection"""
        try:
            # Load YOLOv4 or YOLOv5 model
            # In a real implementation, you would download these files:
            # wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
            # wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4.cfg
            # wget https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names
            
            config_path = "models/yolov4.cfg"
            weights_path = "models/yolov4.weights"
            classes_path = "models/coco.names"
            
            # For demonstration, we'll use a simple cascade classifier
            # In production, replace with actual YOLO model
            self.net = None  # cv2.dnn.readNet(weights_path, config_path)
            self.use_yolo = False
            
            # Load class names
            try:
                with open(classes_path, 'r') as f:
                    self.class_names = [line.strip() for line in f.readlines()]
            except:
                # Default COCO classes
                self.class_names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 
                                  'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
                                  'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
                                  'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe']
            
            # Load alternative detectors
            self.load_alternative_detectors()
            
            self.get_logger().info("Object detection models loaded")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.load_alternative_detectors()
    
    def load_alternative_detectors(self):
        """Load alternative detection methods when YOLO is not available"""
        try:
            # Load Haar cascades for basic detection
            self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            self.body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')
            
            # Initialize background subtractor for motion detection
            self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=True)
            
            self.get_logger().info("Alternative detection methods loaded")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load alternative detectors: {e}")
    
    def camera_callback(self, msg: Image):
        """Process camera image for object detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Perform object detection
            detections = self.detect_objects(cv_image)
            
            # Update trackers if enabled
            if self.enable_tracking:
                detections = self.update_tracking(cv_image, detections)
            
            # Publish detections
            self.publish_detections(detections, msg.header)
            
            # Check for alerts
            self.check_alerts(detections)
            
            # Publish debug image
            if self.publish_debug:
                debug_image = self.draw_detections(cv_image, detections)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """Detect objects in the image"""
        detections = []
        
        if self.use_yolo and self.net is not None:
            detections = self.detect_with_yolo(image)
        else:
            detections = self.detect_with_alternatives(image)
        
        return detections
    
    def detect_with_yolo(self, image: np.ndarray) -> List[Dict]:
        """Detect objects using YOLO model"""
        detections = []
        
        # Create blob from image
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        
        # Run inference
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(output_layers)
        
        # Process outputs
        boxes = []
        confidences = []
        class_ids = []
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > self.confidence_threshold and class_id in self.classes_of_interest:
                    # Scale bounding box back to image size
                    center_x = int(detection[0] * image.shape[1])
                    center_y = int(detection[1] * image.shape[0])
                    width = int(detection[2] * image.shape[1])
                    height = int(detection[3] * image.shape[0])
                    
                    x = int(center_x - width / 2)
                    y = int(center_y - height / 2)
                    
                    boxes.append([x, y, width, height])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        # Apply Non-Maximum Suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)
        
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                detections.append({
                    'class_id': class_ids[i],
                    'class_name': self.class_names[class_ids[i]],
                    'confidence': confidences[i],
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2)
                })
        
        return detections
    
    def detect_with_alternatives(self, image: np.ndarray) -> List[Dict]:
        """Detect objects using alternative methods"""
        detections = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Face detection
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        for (x, y, w, h) in faces:
            detections.append({
                'class_id': 0,
                'class_name': 'person',
                'confidence': 0.8,
                'bbox': (x, y, w, h),
                'center': (x + w//2, y + h//2),
                'detection_method': 'face_cascade'
            })
        
        # Full body detection
        bodies = self.body_cascade.detectMultiScale(gray, 1.1, 4)
        for (x, y, w, h) in bodies:
            # Check if this overlaps with a face detection
            overlap = False
            for face_det in detections:
                if self.calculate_overlap((x, y, w, h), face_det['bbox']) > 0.3:
                    overlap = True
                    break
            
            if not overlap:
                detections.append({
                    'class_id': 0,
                    'class_name': 'person',
                    'confidence': 0.7,
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2),
                    'detection_method': 'body_cascade'
                })
        
        # Motion-based detection for animals
        motion_detections = self.detect_motion_objects(image)
        detections.extend(motion_detections)
        
        return detections
    
    def detect_motion_objects(self, image: np.ndarray) -> List[Dict]:
        """Detect moving objects that could be animals"""
        detections = []
        
        # Apply background subtraction
        fg_mask = self.bg_subtractor.apply(image)
        
        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if 500 < area < 50000:  # Filter by size
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate aspect ratio to filter out non-animal shapes
                aspect_ratio = w / h
                if 0.3 < aspect_ratio < 3.0:  # Reasonable aspect ratio for animals
                    confidence = min(area / 10000.0, 0.6)  # Lower confidence for motion
                    
                    detections.append({
                        'class_id': 14,  # Generic 'animal' class
                        'class_name': 'animal',
                        'confidence': confidence,
                        'bbox': (x, y, w, h),
                        'center': (x + w//2, y + h//2),
                        'detection_method': 'motion'
                    })
        
        return detections
    
    def update_tracking(self, image: np.ndarray, detections: List[Dict]) -> List[Dict]:
        """Update object tracking"""
        # Update existing trackers
        updated_trackers = {}
        for tracker_id, tracker in self.trackers.items():
            success, bbox = tracker['tracker'].update(image)
            if success:
                tracker['bbox'] = bbox
                tracker['age'] += 1
                if tracker['age'] < 30:  # Keep tracker for 30 frames
                    updated_trackers[tracker_id] = tracker
        
        self.trackers = updated_trackers
        
        # Associate detections with existing trackers
        tracked_detections = []
        unmatched_detections = []
        
        for detection in detections:
            best_match = None
            best_overlap = 0
            
            for tracker_id, tracker in self.trackers.items():
                overlap = self.calculate_overlap(detection['bbox'], tracker['bbox'])
                if overlap > best_overlap and overlap > 0.3:
                    best_overlap = overlap
                    best_match = tracker_id
            
            if best_match is not None:
                # Update tracker with detection
                detection['tracker_id'] = best_match
                self.trackers[best_match]['bbox'] = detection['bbox']
                self.trackers[best_match]['age'] = 0
                tracked_detections.append(detection)
            else:
                unmatched_detections.append(detection)
        
        # Create new trackers for unmatched detections
        for detection in unmatched_detections:
            tracker = cv2.TrackerCSRT_create()
            x, y, w, h = detection['bbox']
            if tracker.init(image, (x, y, w, h)):
                tracker_id = self.next_tracker_id
                self.next_tracker_id += 1
                
                self.trackers[tracker_id] = {
                    'tracker': tracker,
                    'bbox': detection['bbox'],
                    'class_name': detection['class_name'],
                    'age': 0
                }
                
                detection['tracker_id'] = tracker_id
                tracked_detections.append(detection)
        
        return tracked_detections
    
    def calculate_overlap(self, bbox1: Tuple[int, int, int, int], bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate overlap between two bounding boxes"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # Calculate intersection
        x_left = max(x1, x2)
        y_top = max(y1, y2)
        x_right = min(x1 + w1, x2 + w2)
        y_bottom = min(y1 + h1, y2 + h2)
        
        if x_right <= x_left or y_bottom <= y_top:
            return 0.0
        
        intersection = (x_right - x_left) * (y_bottom - y_top)
        union = w1 * h1 + w2 * h2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def publish_detections(self, detections: List[Dict], header):
        """Publish detections as ROS message"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        for det in detections:
            detection_msg = Detection2D()
            
            # Bounding box
            x, y, w, h = det['bbox']
            detection_msg.bbox.center.x = float(x + w/2)
            detection_msg.bbox.center.y = float(y + h/2)
            detection_msg.bbox.size_x = float(w)
            detection_msg.bbox.size_y = float(h)
            
            # Hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(det['class_id'])
            hypothesis.score = det['confidence']
            detection_msg.results = [hypothesis]
            
            detection_array.detections.append(detection_msg)
        
        self.detections_pub.publish(detection_array)
    
    def check_alerts(self, detections: List[Dict]):
        """Check for alert conditions"""
        alerts = []
        
        # Check for people in dangerous areas
        people_count = sum(1 for det in detections if det['class_name'] == 'person')
        if people_count > 0:
            alerts.append(f"Detected {people_count} person(s) in the area")
        
        # Check for vehicles
        vehicles = [det for det in detections if det['class_name'] in ['car', 'truck', 'motorcycle', 'bus']]
        if vehicles:
            alerts.append(f"Detected {len(vehicles)} vehicle(s)")
        
        # Check for wildlife
        wildlife = [det for det in detections if det['class_name'] in ['bear', 'elephant', 'horse']]
        if wildlife:
            for animal in wildlife:
                alerts.append(f"Large animal detected: {animal['class_name']}")
        
        # Publish alerts
        for alert in alerts:
            alert_msg = String()
            alert_msg.data = json.dumps({
                'timestamp': self.get_clock().now().to_msg(),
                'alert': alert,
                'type': 'object_detection'
            }, default=str)
            self.alerts_pub.publish(alert_msg)
            self.get_logger().info(f"ALERT: {alert}")
    
    def draw_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """Draw detection results on image"""
        debug_image = image.copy()
        
        for det in detections:
            x, y, w, h = det['bbox']
            class_name = det['class_name']
            confidence = det['confidence']
            
            # Color based on class
            if class_name == 'person':
                color = (0, 255, 0)  # Green for people
            elif class_name in ['car', 'truck', 'motorcycle', 'bus']:
                color = (255, 0, 0)  # Blue for vehicles
            elif class_name in ['bear', 'elephant']:
                color = (0, 0, 255)  # Red for dangerous animals
            else:
                color = (255, 255, 0)  # Cyan for other animals
            
            # Draw bounding box
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            if 'tracker_id' in det:
                label += f" ID:{det['tracker_id']}"
            
            cv2.putText(debug_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return debug_image


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
