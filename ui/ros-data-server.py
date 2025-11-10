#!/usr/bin/env python3
"""
Simple ROS Bridge for Fire Warden Bot UI
Connects ROS 2 topics to HTTP endpoints for the web interface
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS message types
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool, Float32

import json
import threading
import time
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
import base64
import cv2
from cv_bridge import CvBridge


class ROSBridge(Node):
    def __init__(self):
        super().__init__('ros_bridge_server')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Data storage
        self.data = {
            'odometry': None,
            'cmd_vel': None,
            'image': None,
            'laser': None,
            'imu': None,
            'battery': 85.0,  # Mock battery level
            'mission_status': 'idle',
            'leaf_detections': None,
            'system_status': {
                'ros_connected': True,
                'simulation_running': True,
                'navigation_active': False,
                'vision_active': False,
                'timestamp': datetime.now().isoformat()
            }
        }
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscribers
        self.setup_subscribers()
        
        # Create publishers for UI commands
        self.setup_publishers()
        
        # Start HTTP server
        self.start_http_server()
        
        self.get_logger().info('🔥 ROS Bridge Server started on http://localhost:8090')
    
    def setup_subscribers(self):
        """Create ROS subscribers"""
        
        # Odometry - try multiple possible topics
        odometry_topics = ['/model/drone1/odometry', '/odom', '/odometry/filtered']
        for topic in odometry_topics:
            try:
                self.odom_sub = self.create_subscription(
                    Odometry, topic, self.odometry_callback, self.reliable_qos
                )
                self.get_logger().info(f'Subscribed to odometry: {topic}')
                break
            except:
                continue
        
        # Command velocity
        cmd_vel_topics = ['/model/drone1/cmd_vel', '/cmd_vel']
        for topic in cmd_vel_topics:
            try:
                self.cmd_vel_sub = self.create_subscription(
                    Twist, topic, self.cmd_vel_callback, self.best_effort_qos
                )
                self.get_logger().info(f'Subscribed to cmd_vel: {topic}')
                break
            except:
                continue
        
        # Camera image
        image_topics = ['/model/drone1/camera', '/camera/image_raw', '/leaf_detector/annotated_image']
        for topic in image_topics:
            try:
                self.image_sub = self.create_subscription(
                    Image, topic, self.image_callback, self.best_effort_qos
                )
                self.get_logger().info(f'Subscribed to camera: {topic}')
                break
            except:
                continue
        
        # Laser scan
        laser_topics = ['/model/drone1/scan', '/scan']
        for topic in laser_topics:
            try:
                self.laser_sub = self.create_subscription(
                    LaserScan, topic, self.laser_callback, self.best_effort_qos
                )
                self.get_logger().info(f'Subscribed to laser: {topic}')
                break
            except:
                continue
        
        # IMU
        imu_topics = ['/model/drone1/imu', '/imu/data']
        for topic in imu_topics:
            try:
                self.imu_sub = self.create_subscription(
                    Imu, topic, self.imu_callback, self.best_effort_qos
                )
                self.get_logger().info(f'Subscribed to IMU: {topic}')
                break
            except:
                continue
        
        # Leaf detections
        try:
            self.leaf_sub = self.create_subscription(
                String, '/leaf_detections', self.leaf_callback, self.reliable_qos
            )
            self.get_logger().info('Subscribed to leaf detections')
        except:
            pass
        
        # Mission status
        try:
            self.mission_sub = self.create_subscription(
                String, '/mission/status', self.mission_status_callback, self.reliable_qos
            )
            self.get_logger().info('Subscribed to mission status')
        except:
            pass
    
    def setup_publishers(self):
        """Create ROS publishers for UI commands"""
        
        self.mission_goal_pub = self.create_publisher(
            PoseStamped, '/mission/goal', 10
        )
        
        self.mission_cmd_pub = self.create_publisher(
            String, '/mission/command', 10
        )
        
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/emergency_stop', 10
        )
    
    # Callback functions
    def odometry_callback(self, msg):
        """Process odometry data"""
        self.data['odometry'] = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular_velocity': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            },
            'timestamp': datetime.now().isoformat()
        }
    
    def cmd_vel_callback(self, msg):
        """Process velocity commands"""
        self.data['cmd_vel'] = {
            'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
            'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z},
            'timestamp': datetime.now().isoformat()
        }
    
    def image_callback(self, msg):
        """Process camera images"""
        try:
            # Convert to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Resize for web (max 640px width)
            height, width = cv_image.shape[:2]
            if width > 640:
                scale = 640.0 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
            
            # Encode as JPEG and base64
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            
            self.data['image'] = {
                'data': img_base64,
                'width': cv_image.shape[1],
                'height': cv_image.shape[0],
                'encoding': 'jpeg',
                'timestamp': datetime.now().isoformat()
            }
            
            # Update vision active status
            self.data['system_status']['vision_active'] = True
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
    
    def laser_callback(self, msg):
        """Process laser scan data"""
        # Sample every 5th point to reduce data size
        ranges = list(msg.ranges)[::5]
        
        self.data['laser'] = {
            'ranges': ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment * 5,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'timestamp': datetime.now().isoformat()
        }
    
    def imu_callback(self, msg):
        """Process IMU data"""
        self.data['imu'] = {
            'orientation': {
                'x': msg.orientation.x, 'y': msg.orientation.y,
                'z': msg.orientation.z, 'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'timestamp': datetime.now().isoformat()
        }
    
    def leaf_callback(self, msg):
        """Process leaf detection data"""
        try:
            leaf_data = json.loads(msg.data)
            self.data['leaf_detections'] = leaf_data
            self.data['system_status']['vision_active'] = True
        except json.JSONDecodeError:
            self.data['leaf_detections'] = {
                'raw_message': msg.data,
                'timestamp': datetime.now().isoformat()
            }
    
    def mission_status_callback(self, msg):
        """Process mission status"""
        self.data['mission_status'] = msg.data
        
        # Update navigation active status
        nav_active = msg.data in ['executing', 'planning', 'navigating']
        self.data['system_status']['navigation_active'] = nav_active
    
    def start_http_server(self):
        """Start HTTP server for UI communication"""
        
        class RequestHandler(BaseHTTPRequestHandler):
            def __init__(self, ros_bridge, *args, **kwargs):
                self.ros_bridge = ros_bridge
                super().__init__(*args, **kwargs)
            
            def do_GET(self):
                """Handle GET requests"""
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()
                
                path = self.path
                response = {'connected': True}
                
                if path == '/api/status':
                    response.update({
                        'system_status': self.ros_bridge.data['system_status']
                    })
                elif path == '/api/odometry':
                    response.update({
                        'odometry': self.ros_bridge.data['odometry']
                    })
                elif path == '/api/sensors':
                    response.update({
                        'imu': self.ros_bridge.data['imu'],
                        'laser': self.ros_bridge.data['laser']
                    })
                elif path == '/api/image':
                    response.update({
                        'image': self.ros_bridge.data['image']
                    })
                elif path == '/api/vision':
                    response.update({
                        'leaf_detections': self.ros_bridge.data['leaf_detections']
                    })
                elif path == '/api/mission':
                    response.update({
                        'status': {'status': self.ros_bridge.data['mission_status']},
                        'battery': {'level': self.ros_bridge.data['battery']}
                    })
                elif path == '/api/all':
                    response.update({
                        'data': self.ros_bridge.data
                    })
                else:
                    response = {'error': 'Unknown endpoint', 'connected': True}
                
                self.wfile.write(json.dumps(response).encode())
            
            def do_POST(self):
                """Handle POST requests for commands"""
                try:
                    content_length = int(self.headers['Content-Length'])
                    post_data = self.rfile.read(content_length)
                    command = json.loads(post_data.decode())
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    
                    # Process command
                    response = self.ros_bridge.handle_command(command)
                    self.wfile.write(json.dumps(response).encode())
                    
                except Exception as e:
                    self.send_response(400)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(json.dumps({'error': str(e)}).encode())
            
            def do_OPTIONS(self):
                """Handle CORS preflight"""
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()
            
            def log_message(self, format, *args):
                """Suppress request logging"""
                pass
        
        # Create handler with ROS bridge reference
        def handler_factory(*args, **kwargs):
            return RequestHandler(self, *args, **kwargs)
        
        # Start server in background thread
        def run_server():
            try:
                server = HTTPServer(('localhost', 8090), handler_factory)
                server.serve_forever()
            except Exception as e:
                self.get_logger().error(f'HTTP server error: {str(e)}')
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
    
    def handle_command(self, command):
        """Handle commands from UI"""
        try:
            cmd_type = command.get('type')
            
            if cmd_type == 'mission_goal':
                # Send mission goal
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                
                pose = command.get('pose', {})
                pos = pose.get('position', {})
                ori = pose.get('orientation', {})
                
                goal_msg.pose.position.x = float(pos.get('x', 0.0))
                goal_msg.pose.position.y = float(pos.get('y', 0.0))
                goal_msg.pose.position.z = float(pos.get('z', 0.0))
                goal_msg.pose.orientation.x = float(ori.get('x', 0.0))
                goal_msg.pose.orientation.y = float(ori.get('y', 0.0))
                goal_msg.pose.orientation.z = float(ori.get('z', 0.0))
                goal_msg.pose.orientation.w = float(ori.get('w', 1.0))
                
                self.mission_goal_pub.publish(goal_msg)
                return {'success': True, 'message': 'Mission goal sent'}
            
            elif cmd_type == 'mission_command':
                # Send mission command
                cmd_msg = String()
                cmd_msg.data = command.get('command', '')
                self.mission_cmd_pub.publish(cmd_msg)
                return {'success': True, 'message': f'Command sent: {cmd_msg.data}'}
            
            elif cmd_type == 'emergency_stop':
                # Emergency stop
                stop_msg = Bool()
                stop_msg.data = True
                self.emergency_stop_pub.publish(stop_msg)
                return {'success': True, 'message': 'Emergency stop activated'}
            
            else:
                return {'success': False, 'message': f'Unknown command: {cmd_type}'}
                
        except Exception as e:
            return {'success': False, 'message': f'Command error: {str(e)}'}


def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = ROSBridge()
        
        # Print available endpoints
        print("📊 ROS Bridge API Endpoints:")
        print("   • GET  /api/status     - System status")
        print("   • GET  /api/odometry   - Robot position & velocity")
        print("   • GET  /api/sensors    - IMU & laser data")
        print("   • GET  /api/image      - Camera feed")
        print("   • GET  /api/vision     - Leaf detection results")
        print("   • GET  /api/mission    - Mission status & battery")
        print("   • GET  /api/all        - All data combined")
        print("   • POST /api/command    - Send commands to robot")
        print()
        print("🌐 Access UI at: http://localhost:9000")
        print("🔗 Bridge running at: http://localhost:8090")
        print()
        
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print('\n🛑 ROS Bridge shutting down...')
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
