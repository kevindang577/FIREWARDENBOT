"""
Launch file for Fire Warden Bot computer vision system
Starts all vision nodes for fire detection and object tracking
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='husky',
        description='Type of robot: husky or drone',
        choices=['husky', 'drone']
    )
    
    enable_fire_detection_arg = DeclareLaunchArgument(
        'enable_fire_detection',
        default_value='true',
        description='Enable fire detection'
    )
    
    enable_object_detection_arg = DeclareLaunchArgument(
        'enable_object_detection',
        default_value='true',
        description='Enable object detection and tracking'
    )
    
    enable_stereo_vision_arg = DeclareLaunchArgument(
        'enable_stereo_vision',
        default_value='false',
        description='Enable stereo vision processing'
    )
    
    use_cpp_detector_arg = DeclareLaunchArgument(
        'use_cpp_detector',
        default_value='true',
        description='Use C++ fast fire detector instead of Python'
    )
    
    # Get launch configurations
    robot_type = LaunchConfiguration('robot_type')
    enable_fire_detection = LaunchConfiguration('enable_fire_detection')
    enable_object_detection = LaunchConfiguration('enable_object_detection')
    enable_stereo_vision = LaunchConfiguration('enable_stereo_vision')
    use_cpp_detector = LaunchConfiguration('use_cpp_detector')
    
    # Define camera topics based on robot type
    husky_camera_topic = '/husky/camera/image_raw'
    husky_thermal_topic = '/husky/thermal_camera/image_raw'
    drone_camera_topic = '/drone/camera/image_raw'
    drone_thermal_topic = '/drone/thermal_camera/image_raw'
    
    # Husky Vision Nodes
    husky_vision_group = GroupAction([
        # Fire Detection (Python version)
        Node(
            package='firewardenbot_vision',
            executable='fire_detection_node.py',
            name='husky_fire_detection',
            namespace='husky',
            parameters=[{
                'camera_topic': husky_camera_topic,
                'thermal_topic': husky_thermal_topic,
                'detection_method': 'combined',
                'confidence_threshold': 0.7,
                'publish_debug_images': True,
                'roi_enabled': True
            }],
            condition=IfCondition([enable_fire_detection, ' and not ', use_cpp_detector]),
            remappings=[
                ('/fire_detection/alert', '/husky/fire_detection/alert'),
                ('/fire_detection/location', '/husky/fire_detection/location'),
                ('/fire_detection/detected', '/husky/fire_detection/detected'),
                ('/fire_detection/debug_image', '/husky/fire_detection/debug_image')
            ]
        ),
        
        # Fire Detection (C++ version)
        Node(
            package='firewardenbot_vision',
            executable='fast_fire_detector',
            name='husky_fast_fire_detector',
            namespace='husky',
            parameters=[{
                'camera_topic': husky_camera_topic,
                'confidence_threshold': 0.7,
                'min_fire_area': 500,
                'max_fire_area': 50000
            }],
            condition=IfCondition([enable_fire_detection, ' and ', use_cpp_detector]),
            remappings=[
                ('/fire_detection/alert', '/husky/fire_detection/alert'),
                ('/fire_detection/location', '/husky/fire_detection/location'),
                ('/fire_detection/detected', '/husky/fire_detection/detected'),
                ('/fire_detection/debug_image', '/husky/fire_detection/debug_image')
            ]
        ),
        
        # Object Detection and Tracking
        Node(
            package='firewardenbot_vision',
            executable='object_detection_node.py',
            name='husky_object_detection',
            namespace='husky',
            parameters=[{
                'camera_topic': husky_camera_topic,
                'detection_confidence': 0.5,
                'nms_threshold': 0.4,
                'enable_tracking': True,
                'publish_debug_images': True
            }],
            condition=IfCondition(enable_object_detection),
            remappings=[
                ('/object_detection/detections', '/husky/object_detection/detections'),
                ('/object_detection/alerts', '/husky/object_detection/alerts'),
                ('/object_detection/debug_image', '/husky/object_detection/debug_image')
            ]
        ),
        
        # Stereo Vision Processing
        Node(
            package='firewardenbot_vision',
            executable='stereo_vision_node.py',
            name='husky_stereo_vision',
            namespace='husky',
            parameters=[{
                'left_camera_topic': '/husky/camera_left/image_raw',
                'right_camera_topic': '/husky/camera_right/image_raw',
                'publish_point_cloud': True,
                'publish_depth_image': True
            }],
            condition=IfCondition(enable_stereo_vision),
            remappings=[
                ('/stereo_vision/point_cloud', '/husky/stereo_vision/point_cloud'),
                ('/stereo_vision/depth_image', '/husky/stereo_vision/depth_image')
            ]
        )
    ], condition=IfCondition(['$(eval "\'', robot_type, '\' == \'husky\'")']))
    
    # Drone Vision Nodes
    drone_vision_group = GroupAction([
        # Fire Detection (Python version)
        Node(
            package='firewardenbot_vision',
            executable='fire_detection_node.py',
            name='drone_fire_detection',
            namespace='drone',
            parameters=[{
                'camera_topic': drone_camera_topic,
                'thermal_topic': drone_thermal_topic,
                'detection_method': 'combined',
                'confidence_threshold': 0.7,
                'publish_debug_images': True,
                'roi_enabled': False  # Drone covers wider area
            }],
            condition=IfCondition([enable_fire_detection, ' and not ', use_cpp_detector]),
            remappings=[
                ('/fire_detection/alert', '/drone/fire_detection/alert'),
                ('/fire_detection/location', '/drone/fire_detection/location'),
                ('/fire_detection/detected', '/drone/fire_detection/detected'),
                ('/fire_detection/debug_image', '/drone/fire_detection/debug_image')
            ]
        ),
        
        # Fire Detection (C++ version)
        Node(
            package='firewardenbot_vision',
            executable='fast_fire_detector',
            name='drone_fast_fire_detector',
            namespace='drone',
            parameters=[{
                'camera_topic': drone_camera_topic,
                'confidence_threshold': 0.7,
                'min_fire_area': 300,  # Smaller minimum for aerial view
                'max_fire_area': 100000
            }],
            condition=IfCondition([enable_fire_detection, ' and ', use_cpp_detector]),
            remappings=[
                ('/fire_detection/alert', '/drone/fire_detection/alert'),
                ('/fire_detection/location', '/drone/fire_detection/location'),
                ('/fire_detection/detected', '/drone/fire_detection/detected'),
                ('/fire_detection/debug_image', '/drone/fire_detection/debug_image')
            ]
        ),
        
        # Object Detection and Tracking
        Node(
            package='firewardenbot_vision',
            executable='object_detection_node.py',
            name='drone_object_detection',
            namespace='drone',
            parameters=[{
                'camera_topic': drone_camera_topic,
                'detection_confidence': 0.4,  # Lower threshold for aerial view
                'nms_threshold': 0.4,
                'enable_tracking': True,
                'publish_debug_images': True
            }],
            condition=IfCondition(enable_object_detection),
            remappings=[
                ('/object_detection/detections', '/drone/object_detection/detections'),
                ('/object_detection/alerts', '/drone/object_detection/alerts'),
                ('/object_detection/debug_image', '/drone/object_detection/debug_image')
            ]
        ),
        
        # Stereo Vision Processing
        Node(
            package='firewardenbot_vision',
            executable='stereo_vision_node.py',
            name='drone_stereo_vision',
            namespace='drone',
            parameters=[{
                'left_camera_topic': '/drone/camera_left/image_raw',
                'right_camera_topic': '/drone/camera_right/image_raw',
                'publish_point_cloud': True,
                'publish_depth_image': True
            }],
            condition=IfCondition(enable_stereo_vision),
            remappings=[
                ('/stereo_vision/point_cloud', '/drone/stereo_vision/point_cloud'),
                ('/stereo_vision/depth_image', '/drone/stereo_vision/depth_image')
            ]
        )
    ], condition=IfCondition(['$(eval "\'', robot_type, '\' == \'drone\'")']))
    
    # Image Processing Node (shared)
    image_processing_node = Node(
        package='firewardenbot_vision',
        executable='image_processing_node.py',
        name='image_processing',
        parameters=[{
            'enable_enhancement': True,
            'enable_noise_reduction': True,
            'contrast_enhancement': 1.2,
            'brightness_adjustment': 0.1
        }]
    )
    
    return LaunchDescription([
        robot_type_arg,
        enable_fire_detection_arg,
        enable_object_detection_arg,
        enable_stereo_vision_arg,
        use_cpp_detector_arg,
        husky_vision_group,
        drone_vision_group,
        image_processing_node
    ])
