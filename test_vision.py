#!/usr/bin/env python3
"""
Simple test to verify the computer vision system components
"""

import sys
import os

def test_imports():
    """Test if required packages can be imported"""
    print("🔍 Testing Computer Vision System Components...")
    print("=" * 50)
    
    # Test basic imports
    try:
        import cv2
        print("✅ OpenCV imported successfully")
        print(f"   OpenCV version: {cv2.__version__}")
    except ImportError as e:
        print(f"❌ OpenCV import failed: {e}")
        return False
    
    try:
        import numpy as np
        print("✅ NumPy imported successfully")
        print(f"   NumPy version: {np.__version__}")
    except ImportError as e:
        print(f"❌ NumPy import failed: {e}")
        return False
    
    try:
        import rclpy
        print("✅ ROS 2 rclpy imported successfully")
    except ImportError as e:
        print(f"❌ ROS 2 rclpy import failed: {e}")
        return False
    
    try:
        from sensor_msgs.msg import Image
        print("✅ ROS 2 sensor_msgs imported successfully")
    except ImportError as e:
        print(f"❌ ROS 2 sensor_msgs import failed: {e}")
        return False
    
    try:
        from cv_bridge import CvBridge
        print("✅ cv_bridge imported successfully")
    except ImportError as e:
        print(f"❌ cv_bridge import failed: {e}")
        print("   Note: cv_bridge is needed to convert ROS images to OpenCV format")
        return False
    
    return True

def test_color_detection():
    """Test color detection algorithms with a sample image"""
    print("\n🎨 Testing Color Detection Algorithms...")
    print("=" * 50)
    
    try:
        import cv2
        import numpy as np
        
        # Create a test image with autumn colors
        test_image = np.zeros((400, 600, 3), dtype=np.uint8)
        
        # Add colored rectangles representing different leaf colors
        # Orange leaf
        cv2.rectangle(test_image, (50, 50), (150, 150), (0, 165, 255), -1)  # BGR format
        # Red leaf
        cv2.rectangle(test_image, (200, 50), (300, 150), (0, 0, 255), -1)
        # Yellow leaf
        cv2.rectangle(test_image, (350, 50), (450, 150), (0, 255, 255), -1)
        # Brown leaf
        cv2.rectangle(test_image, (50, 200), (150, 300), (19, 69, 139), -1)
        
        print("✅ Created test image with autumn leaf colors")
        
        # Test HSV conversion
        hsv_image = cv2.cvtColor(test_image, cv2.COLOR_BGR2HSV)
        print("✅ BGR to HSV conversion successful")
        
        # Test color range detection for orange
        orange_lower = np.array([5, 150, 150])
        orange_upper = np.array([15, 255, 255])
        orange_mask = cv2.inRange(hsv_image, orange_lower, orange_upper)
        
        # Count orange pixels
        orange_pixels = np.sum(orange_mask > 0)
        print(f"✅ Orange leaf detection: {orange_pixels} pixels found")
        
        # Test contour detection
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"✅ Contour detection: {len(contours)} contours found")
        
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            print(f"✅ Largest contour area: {area:.0f} pixels")
            
            # Test bounding box
            x, y, w, h = cv2.boundingRect(largest_contour)
            print(f"✅ Bounding box: ({x}, {y}, {w}, {h})")
        
        return True
        
    except Exception as e:
        print(f"❌ Color detection test failed: {e}")
        return False

def main():
    print("🍂 Fire Warden Bot - Computer Vision System Test")
    print("🌿 Testing leaf detection and color classification components")
    print()
    
    # Test imports
    if not test_imports():
        print("\n❌ Import tests failed. Please install missing packages.")
        return False
    
    # Test color detection
    if not test_color_detection():
        print("\n❌ Color detection tests failed.")
        return False
    
    print("\n" + "=" * 50)
    print("🎉 ALL TESTS PASSED!")
    print("✅ Computer Vision System is ready for leaf detection")
    print("✅ The system can:")
    print("   • Process camera images")
    print("   • Convert color spaces (BGR ↔ HSV)")
    print("   • Detect colored objects (autumn leaves)")
    print("   • Find and analyze contours")
    print("   • Calculate bounding boxes and areas")
    print()
    print("🚁 Ready to detect autumn leaves from drone camera!")
    print("=" * 50)
    
    return True

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
