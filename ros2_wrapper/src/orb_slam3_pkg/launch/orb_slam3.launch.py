from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument('video_path',
        default_value='/orbslam_playground/webcam_data/seq2_30fps.mp4',
        description='Path to video file'),
      DeclareLaunchArgument('vocab_path',
        default_value='/ORB_SLAM3_ROS2/Vocabulary/ORBvoc.txt',
        description='ORB-SLAM3 vocabulary file'),
      DeclareLaunchArgument('settings_path',
        default_value='/ORB_SLAM3_ROS2/Examples/Monocular/webcam_new.yaml',
        description='ORB-SLAM3 settings file'),

      # Video publisher node
      Node(
        package='orb_slam3_pkg',
        executable='video_publisher_node',
        name='video_publisher',
        output='screen',
        parameters=[{
          'video_path': LaunchConfiguration('video_path'),
          'camera_name': 'video_cam',
          'queue_size': 10,
        }]
      ),

      # ORB-SLAM3 node
      Node(
        package='orb_slam3_pkg',
        executable='orb_slam3_node',
        name='orb_slam3_node',
        output='screen',
        parameters=[{
          'vocab_path': LaunchConfiguration('vocab_path'),
          'settings_path': LaunchConfiguration('settings_path'),
          'image_topic': '/camera/image_raw',
        }]
      ),
    ])
