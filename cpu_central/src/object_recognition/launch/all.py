# =============================================================================
#  Header
# =============================================================================

from launch import LaunchDescription
from launch_ros.actions import Node

# =============================================================================
#  Launch Description
# =============================================================================

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_recognition',
            executable='webcam_publisher.py',
            name='webcam_publisher'
        ),
        Node(
            package='object_recognition',
            executable='yolo_node.py',
            name='yolo_node'
        ),
        Node(
            package='object_recognition',
            executable='yolo_subscriber.py',
            name='yolo_subscriber'
        )
    ])