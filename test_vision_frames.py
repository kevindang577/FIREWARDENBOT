#!/usr/bin/env python3
"""
Test Vision System with Sample Images
Generates test frames with simulated leaf detections
"""

import cv2
import numpy as np
import os
import json
from datetime import datetime

def create_test_frame_with_leaves():
    """Create a test frame with simulated autumn leaves"""
    
    # Create a forest background
    frame = np.ones((480, 640, 3), dtype=np.uint8) * 40  # Dark forest background
    
    # Add some forest texture
    for _ in range(100):
        x, y = np.random.randint(0, 640), np.random.randint(0, 480)
        cv2.circle(frame, (x, y), np.random.randint(1, 3), (20, 60, 20), -1)
    
    # Add simulated autumn leaves with different colors
    leaf_detections = []
    
    # Orange leaf
    orange_center = (150, 200)
    cv2.ellipse(frame, orange_center, (25, 15), 45, 0, 360, (30, 165, 255), -1)  # BGR for orange
    leaf_detections.append({
        'color': 'bright_orange',
        'center': orange_center,
        'area': 1178,
        'confidence': 0.95,
        'bounding_box': [125, 185, 50, 30]
    })
    
    # Yellow-orange leaf
    yellow_center = (350, 150)
    cv2.ellipse(frame, yellow_center, (20, 12), -30, 0, 360, (0, 215, 255), -1)  # BGR for yellow-orange
    leaf_detections.append({
        'color': 'yellow_orange',
        'center': yellow_center,
        'area': 754,
        'confidence': 0.87,
        'bounding_box': [330, 138, 40, 24]
    })
    
    # Red-orange leaf
    red_center = (500, 300)
    cv2.ellipse(frame, red_center, (18, 22), 60, 0, 360, (0, 69, 255), -1)  # BGR for red-orange
    leaf_detections.append({
        'color': 'red_orange',
        'center': red_center,
        'area': 1256,
        'confidence': 0.92,
        'bounding_box': [482, 278, 36, 44]
    })
    
    # Brown leaf
    brown_center = (200, 350)
    cv2.ellipse(frame, brown_center, (15, 25), 0, 0, 360, (19, 69, 139), -1)  # BGR for brown
    leaf_detections.append({
        'color': 'brown',
        'center': brown_center,
        'area': 1178,
        'confidence': 0.73,
        'bounding_box': [185, 325, 30, 50]
    })
    
    return frame, leaf_detections

def annotate_frame(frame, detections):
    """Add detection annotations to the frame"""
    annotated = frame.copy()
    
    for detection in detections:
        center = detection['center']
        bbox = detection['bounding_box']
        color_name = detection['color']
        confidence = detection['confidence']
        
        # Color mapping for visualization
        color_map = {
            'bright_orange': (0, 165, 255),
            'yellow_orange': (0, 215, 255), 
            'red_orange': (0, 69, 255),
            'brown': (19, 69, 139),
            'dark_red': (0, 0, 139)
        }
        
        viz_color = color_map.get(color_name, (0, 255, 0))
        
        # Draw bounding box
        x, y, w, h = bbox
        cv2.rectangle(annotated, (x, y), (x + w, y + h), viz_color, 2)
        
        # Draw center point
        cv2.circle(annotated, center, 5, viz_color, -1)
        
        # Add label
        label = f"{color_name} ({confidence:.2f})"
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        cv2.rectangle(annotated, (x, y - label_size[1] - 10), 
                     (x + label_size[0], y), viz_color, -1)
        cv2.putText(annotated, label, (x, y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Add detection summary
    overlay = annotated.copy()
    cv2.rectangle(overlay, (10, 10), (300, 120), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, annotated, 0.3, 0, annotated)
    
    y_offset = 30
    cv2.putText(annotated, f"Leaf Detections: {len(detections)}", 
               (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Color breakdown
    y_offset += 20
    color_counts = {}
    for det in detections:
        color = det['color']
        color_counts[color] = color_counts.get(color, 0) + 1
    
    for color, count in color_counts.items():
        cv2.putText(annotated, f"{color}: {count}", 
                   (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        y_offset += 15
    
    return annotated

def save_detection_data(frame_num, detections):
    """Save detection data as JSON"""
    detection_data = {
        'timestamp': datetime.now().isoformat(),
        'frame_id': f'frame_{frame_num:04d}',
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
    
    return detection_data

def main():
    print("🍂 Testing Fire Warden Bot Vision System")
    print("🌿 Generating sample detection frames...")
    
    output_dir = "/home/student/git/FIREWARDENBOT/detection_outputs"
    
    # Generate test frames
    for frame_num in range(1, 6):  # Generate 5 test frames
        print(f"📸 Generating frame {frame_num:04d}...")
        
        # Create test frame with leaves
        raw_frame, detections = create_test_frame_with_leaves()
        
        # Add some randomness to each frame
        for detection in detections:
            # Slightly vary positions
            detection['center'] = (
                detection['center'][0] + np.random.randint(-10, 10),
                detection['center'][1] + np.random.randint(-10, 10)
            )
            # Vary confidence slightly
            detection['confidence'] = max(0.5, min(1.0, 
                detection['confidence'] + np.random.uniform(-0.1, 0.1)))
        
        # Create annotated version
        annotated_frame = annotate_frame(raw_frame, detections)
        
        # Save raw frame
        raw_filename = f"{output_dir}/frame_{frame_num:04d}_raw.jpg"
        cv2.imwrite(raw_filename, raw_frame)
        
        # Save annotated frame
        annotated_filename = f"{output_dir}/frame_{frame_num:04d}_annotated.jpg"
        cv2.imwrite(annotated_filename, annotated_frame)
        
        # Save detection data
        detection_data = save_detection_data(frame_num, detections)
        json_filename = f"{output_dir}/frame_{frame_num:04d}_detections.json"
        with open(json_filename, 'w') as f:
            json.dump(detection_data, f, indent=2)
        
        print(f"   ✅ Saved: {raw_filename}")
        print(f"   ✅ Saved: {annotated_filename}")
        print(f"   ✅ Saved: {json_filename}")
        print(f"   🍃 Detected {len(detections)} leaves")
    
    print(f"\n🎉 Vision test complete!")
    print(f"📁 Detection outputs saved to: {output_dir}")
    print(f"🔍 Generated {5} test frames with leaf detections")
    
    # Display summary
    print("\n📊 Detection Summary:")
    sample_frame, sample_detections = create_test_frame_with_leaves()
    color_counts = {}
    for det in sample_detections:
        color = det['color']
        color_counts[color] = color_counts.get(color, 0) + 1
    
    for color, count in color_counts.items():
        print(f"   • {color}: {count} detections per frame")

if __name__ == '__main__':
    main()
