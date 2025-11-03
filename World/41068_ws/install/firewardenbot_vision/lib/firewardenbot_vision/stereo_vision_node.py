#!/usr/bin/env python3
"""
Stereo Vision Node for Fire Warden Bot
Provides depth estimation and 3D point cloud generation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class StereoVisionNode(Node):
    def __init__(self):
        super().__init__('stereo_vision_node')
        
        # Parameters
        self.declare_parameter('left_camera_topic', '/camera_left/image_raw')
        self.declare_parameter('right_camera_topic', '/camera_right/image_raw')
        self.declare_parameter('publish_point_cloud', True)
        self.declare_parameter('publish_depth_image', True)
        self.declare_parameter('min_disparity', 0)
        self.declare_parameter('num_disparities', 64)
        self.declare_parameter('block_size', 15)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Get parameters
        left_topic = self.get_parameter('left_camera_topic').get_parameter_value().string_value
        right_topic = self.get_parameter('right_camera_topic').get_parameter_value().string_value
        self.publish_pc = self.get_parameter('publish_point_cloud').get_parameter_value().bool_value
        self.publish_depth = self.get_parameter('publish_depth_image').get_parameter_value().bool_value
        
        # Stereo parameters
        self.min_disp = self.get_parameter('min_disparity').get_parameter_value().integer_value
        self.num_disp = self.get_parameter('num_disparities').get_parameter_value().integer_value
        self.block_size = self.get_parameter('block_size').get_parameter_value().integer_value
        
        # Initialize stereo matcher
        self.stereo = cv2.StereoBM_create(numDisparities=self.num_disp, blockSize=self.block_size)
        
        # Image storage
        self.left_image = None
        self.right_image = None
        self.left_header = None
        
        # Subscribers
        self.left_sub = self.create_subscription(
            Image, left_topic, self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Image, right_topic, self.right_callback, 10)
        
        # Publishers
        if self.publish_pc:
            self.pointcloud_pub = self.create_publisher(PointCloud2, '/stereo_vision/point_cloud', 10)
        if self.publish_depth:
            self.depth_pub = self.create_publisher(Image, '/stereo_vision/depth_image', 10)
        
        # Camera calibration (placeholder - should be loaded from calibration files)
        self.setup_camera_parameters()
        
        self.get_logger().info("Stereo Vision Node initialized")
    
    def setup_camera_parameters(self):
        """Setup camera calibration parameters"""
        # Placeholder camera parameters (should be loaded from calibration)
        self.focal_length = 525.0  # pixels
        self.baseline = 0.1  # meters (distance between cameras)
        self.cx = 320.0  # principal point x
        self.cy = 240.0  # principal point y
        
        # Camera matrix
        self.camera_matrix = np.array([
            [self.focal_length, 0, self.cx],
            [0, self.focal_length, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Distortion coefficients (assuming rectified images)
        self.dist_coeffs = np.zeros(4, dtype=np.float32)
    
    def left_callback(self, msg: Image):
        """Process left camera image"""
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.left_header = msg.header
            self.process_stereo()
        except Exception as e:
            self.get_logger().error(f"Error processing left image: {e}")
    
    def right_callback(self, msg: Image):
        """Process right camera image"""
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_stereo()
        except Exception as e:
            self.get_logger().error(f"Error processing right image: {e}")
    
    def process_stereo(self):
        """Process stereo image pair"""
        if self.left_image is None or self.right_image is None:
            return
        
        # Convert to grayscale
        left_gray = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)
        
        # Compute disparity
        disparity = self.compute_disparity(left_gray, right_gray)
        
        # Publish depth image
        if self.publish_depth:
            self.publish_depth_image(disparity)
        
        # Publish point cloud
        if self.publish_pc:
            self.publish_point_cloud(disparity, self.left_image)
    
    def compute_disparity(self, left_gray: np.ndarray, right_gray: np.ndarray) -> np.ndarray:
        """Compute disparity map from stereo images"""
        # Compute disparity using StereoBM
        disparity = self.stereo.compute(left_gray, right_gray)
        
        # Convert to float and normalize
        disparity = disparity.astype(np.float32) / 16.0
        
        # Filter invalid disparities
        disparity[disparity <= self.min_disp] = 0
        disparity[disparity >= self.num_disp] = 0
        
        return disparity
    
    def publish_depth_image(self, disparity: np.ndarray):
        """Publish depth image"""
        # Convert disparity to depth
        # depth = (focal_length * baseline) / disparity
        depth = np.zeros_like(disparity, dtype=np.float32)
        valid_mask = disparity > 0
        depth[valid_mask] = (self.focal_length * self.baseline) / disparity[valid_mask]
        
        # Convert to millimeters and uint16
        depth_mm = (depth * 1000).astype(np.uint16)
        
        # Create ROS message
        depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
        depth_msg.header = self.left_header
        self.depth_pub.publish(depth_msg)
    
    def publish_point_cloud(self, disparity: np.ndarray, color_image: np.ndarray):
        """Publish 3D point cloud"""
        points = []
        colors = []
        
        h, w = disparity.shape
        
        for v in range(0, h, 4):  # Subsample for performance
            for u in range(0, w, 4):
                d = disparity[v, u]
                if d > 0:
                    # Convert pixel coordinates to 3D
                    z = (self.focal_length * self.baseline) / d
                    x = (u - self.cx) * z / self.focal_length
                    y = (v - self.cy) * z / self.focal_length
                    
                    points.append([x, y, z])
                    
                    # Get color
                    b, g, r = color_image[v, u]
                    colors.append([r, g, b])
        
        if len(points) > 0:
            # Create point cloud message
            points_np = np.array(points, dtype=np.float32)
            colors_np = np.array(colors, dtype=np.uint8)
            
            # Combine points and colors
            cloud_data = []
            for i in range(len(points_np)):
                x, y, z = points_np[i]
                r, g, b = colors_np[i]
                # Pack color into single float
                rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                cloud_data.append([x, y, z, rgb])
            
            # Create PointCloud2 message
            cloud_msg = pc2.create_cloud_xyz_rgb32(self.left_header, cloud_data)
            self.pointcloud_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoVisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
