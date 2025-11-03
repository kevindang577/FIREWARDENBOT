#!/usr/bin/env python3
"""
Vision Aggregator Node for Fire Warden Bot
Combines and processes detection results from multiple vision sources
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray
import json
from typing import Dict, List

class VisionAggregator(Node):
    def __init__(self):
        super().__init__('vision_aggregator')
        
        # Parameters
        self.declare_parameter('robot_type', 'husky')
        self.declare_parameter('aggregate_detections', True)
        self.declare_parameter('publish_combined_alerts', True)
        
        robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        
        # State tracking
        self.fire_detections = {}
        self.object_detections = {}
        self.last_fire_alert = None
        self.last_object_alert = None
        
        # Subscribers for fire detection
        self.fire_alert_sub = self.create_subscription(
            String, f'/{robot_type}/fire_detection/alert', 
            self.fire_alert_callback, 10)
        self.fire_detected_sub = self.create_subscription(
            Bool, f'/{robot_type}/fire_detection/detected', 
            self.fire_detected_callback, 10)
        self.fire_location_sub = self.create_subscription(
            PointStamped, f'/{robot_type}/fire_detection/location',
            self.fire_location_callback, 10)
        
        # Subscribers for object detection
        self.object_detections_sub = self.create_subscription(
            Detection2DArray, f'/{robot_type}/object_detection/detections',
            self.object_detections_callback, 10)
        self.object_alerts_sub = self.create_subscription(
            String, f'/{robot_type}/object_detection/alerts',
            self.object_alerts_callback, 10)
        
        # Publishers for aggregated results
        self.combined_alert_pub = self.create_publisher(String, '/firewardenbot/combined_alerts', 10)
        self.mission_status_pub = self.create_publisher(String, '/firewardenbot/mission_status', 10)
        self.threat_level_pub = self.create_publisher(String, '/firewardenbot/threat_level', 10)
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(5.0, self.publish_status_update)
        
        self.get_logger().info(f"Vision Aggregator initialized for {robot_type}")
    
    def fire_alert_callback(self, msg: String):
        """Process fire detection alerts"""
        try:
            alert_data = json.loads(msg.data)
            self.last_fire_alert = alert_data
            
            # Publish combined alert
            combined_alert = {
                'type': 'fire_detection',
                'severity': 'high',
                'timestamp': alert_data.get('timestamp'),
                'confidence': alert_data.get('confidence', 0),
                'location': alert_data.get('location'),
                'methods': alert_data.get('methods', []),
                'message': f"FIRE DETECTED with {alert_data.get('confidence', 0):.2f} confidence"
            }
            
            self.publish_combined_alert(combined_alert)
            
        except Exception as e:
            self.get_logger().error(f"Error processing fire alert: {e}")
    
    def fire_detected_callback(self, msg: Bool):
        """Track fire detection status"""
        self.fire_detections['active'] = msg.data
        self.fire_detections['last_update'] = self.get_clock().now()
    
    def fire_location_callback(self, msg: PointStamped):
        """Track fire location"""
        self.fire_detections['location'] = {
            'x': msg.point.x,
            'y': msg.point.y,
            'z': msg.point.z,
            'frame': msg.header.frame_id
        }
    
    def object_detections_callback(self, msg: Detection2DArray):
        """Process object detections"""
        detections = []
        for detection in msg.detections:
            if detection.results:
                result = detection.results[0]  # Take highest confidence result
                detections.append({
                    'id': result.id,
                    'confidence': result.score,
                    'bbox': {
                        'x': detection.bbox.center.x - detection.bbox.size_x/2,
                        'y': detection.bbox.center.y - detection.bbox.size_y/2,
                        'width': detection.bbox.size_x,
                        'height': detection.bbox.size_y
                    }
                })
        
        self.object_detections['current'] = detections
        self.object_detections['count'] = len(detections)
        self.object_detections['last_update'] = self.get_clock().now()
    
    def object_alerts_callback(self, msg: String):
        """Process object detection alerts"""
        try:
            alert_data = json.loads(msg.data)
            self.last_object_alert = alert_data
            
            # Determine severity based on alert type
            alert_text = alert_data.get('alert', '').lower()
            if 'bear' in alert_text or 'elephant' in alert_text:
                severity = 'high'
            elif 'person' in alert_text:
                severity = 'medium'
            elif 'vehicle' in alert_text:
                severity = 'medium'
            else:
                severity = 'low'
            
            combined_alert = {
                'type': 'object_detection',
                'severity': severity,
                'timestamp': alert_data.get('timestamp'),
                'alert': alert_data.get('alert'),
                'message': f"Object detected: {alert_data.get('alert')}"
            }
            
            self.publish_combined_alert(combined_alert)
            
        except Exception as e:
            self.get_logger().error(f"Error processing object alert: {e}")
    
    def publish_combined_alert(self, alert_data: Dict):
        """Publish combined alert message"""
        alert_msg = String()
        alert_msg.data = json.dumps(alert_data, default=str)
        self.combined_alert_pub.publish(alert_msg)
        
        # Log based on severity
        message = alert_data.get('message', 'Alert')
        if alert_data.get('severity') == 'high':
            self.get_logger().error(f"HIGH SEVERITY ALERT: {message}")
        elif alert_data.get('severity') == 'medium':
            self.get_logger().warn(f"MEDIUM SEVERITY ALERT: {message}")
        else:
            self.get_logger().info(f"Alert: {message}")
    
    def publish_status_update(self):
        """Publish periodic status updates"""
        current_time = self.get_clock().now()
        
        # Determine overall threat level
        threat_level = self.calculate_threat_level()
        
        # Create status message
        status = {
            'timestamp': current_time.to_msg(),
            'fire_detection_active': self.fire_detections.get('active', False),
            'objects_detected': self.object_detections.get('count', 0),
            'threat_level': threat_level,
            'last_fire_alert': self.last_fire_alert,
            'last_object_alert': self.last_object_alert
        }
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps(status, default=str)
        self.mission_status_pub.publish(status_msg)
        
        # Publish threat level
        threat_msg = String()
        threat_msg.data = threat_level
        self.threat_level_pub.publish(threat_msg)
    
    def calculate_threat_level(self) -> str:
        """Calculate overall threat level"""
        # High threat: Fire detected
        if self.fire_detections.get('active', False):
            return 'HIGH'
        
        # Medium threat: People or large animals detected
        if self.last_object_alert:
            alert_text = self.last_object_alert.get('alert', '').lower()
            if any(animal in alert_text for animal in ['bear', 'elephant', 'person']):
                return 'MEDIUM'
        
        # Low threat: Other objects detected
        if self.object_detections.get('count', 0) > 0:
            return 'LOW'
        
        # No threats detected
        return 'CLEAR'


def main(args=None):
    rclpy.init(args=args)
    node = VisionAggregator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
