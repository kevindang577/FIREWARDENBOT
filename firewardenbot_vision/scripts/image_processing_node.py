#!/usr/bin/env python3
"""
Image Processing Node for Fire Warden Bot
Provides image enhancement, noise reduction, and preprocessing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        
        # Parameters
        self.declare_parameter('enable_enhancement', True)
        self.declare_parameter('enable_noise_reduction', True)
        self.declare_parameter('contrast_enhancement', 1.2)
        self.declare_parameter('brightness_adjustment', 0.1)
        self.declare_parameter('gamma_correction', 1.0)
        self.declare_parameter('gaussian_blur_kernel', 3)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Get parameters
        self.enable_enhancement = self.get_parameter('enable_enhancement').get_parameter_value().bool_value
        self.enable_noise_reduction = self.get_parameter('enable_noise_reduction').get_parameter_value().bool_value
        self.contrast = self.get_parameter('contrast_enhancement').get_parameter_value().double_value
        self.brightness = self.get_parameter('brightness_adjustment').get_parameter_value().double_value
        self.gamma = self.get_parameter('gamma_correction').get_parameter_value().double_value
        self.blur_kernel = self.get_parameter('gaussian_blur_kernel').get_parameter_value().integer_value
        
        # Subscribers (will be remapped for different cameras)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publishers
        self.enhanced_pub = self.create_publisher(Image, '/camera/image_enhanced', 10)
        
        self.get_logger().info("Image Processing Node initialized")
    
    def image_callback(self, msg: Image):
        """Process incoming image"""
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Apply processing pipeline
            processed_image = self.process_image(cv_image)
            
            # Convert back and publish
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header = msg.header
            self.enhanced_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def process_image(self, image: np.ndarray) -> np.ndarray:
        """Apply image processing pipeline"""
        processed = image.copy()
        
        # Noise reduction
        if self.enable_noise_reduction:
            processed = self.reduce_noise(processed)
        
        # Enhancement
        if self.enable_enhancement:
            processed = self.enhance_image(processed)
        
        return processed
    
    def reduce_noise(self, image: np.ndarray) -> np.ndarray:
        """Reduce image noise"""
        # Gaussian blur for noise reduction
        if self.blur_kernel > 0:
            image = cv2.GaussianBlur(image, (self.blur_kernel, self.blur_kernel), 0)
        
        # Bilateral filter for edge-preserving smoothing
        image = cv2.bilateralFilter(image, 9, 75, 75)
        
        return image
    
    def enhance_image(self, image: np.ndarray) -> np.ndarray:
        """Enhance image contrast and brightness"""
        # Convert to float for processing
        image_float = image.astype(np.float32) / 255.0
        
        # Gamma correction
        if self.gamma != 1.0:
            image_float = np.power(image_float, self.gamma)
        
        # Contrast and brightness adjustment
        image_float = image_float * self.contrast + self.brightness
        
        # Clip values and convert back to uint8
        image_float = np.clip(image_float, 0, 1)
        enhanced = (image_float * 255).astype(np.uint8)
        
        return enhanced


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
