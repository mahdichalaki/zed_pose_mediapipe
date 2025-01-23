import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    left_camera_topic_arg = DeclareLaunchArgument(
        'left_camera_topic',
        default_value='/zed/zed_node/left/image_rect_color',
        description='Left camera image topic'
    )

    right_camera_topic_arg = DeclareLaunchArgument(
        'right_camera_topic',
        default_value='/zed/zed_node/right/image_rect_color',
        description='Right camera image topic'
    )

    display_output_arg = DeclareLaunchArgument(
        'display_output',
        default_value='true',
        description='Whether to display output window'
    )
    
    assessment_rate_arg = DeclareLaunchArgument(
        'assessment_rate',
        default_value='20',
        description='Rate at which to assess the pose (in Hz)'
    )


    # Node configuration
    pose_assess_node = Node(
        package='pose_assessment',
        executable='pose_assess',
        name='pose_assess',
        output='screen',
        parameters=[{
            'left_camera_topic': LaunchConfiguration('left_camera_topic'),
            'right_camera_topic': LaunchConfiguration('right_camera_topic'),
            'display_output': LaunchConfiguration('display_output'),
            'assessment_rate': LaunchConfiguration('assessment_rate')
        }]
    )

    return LaunchDescription([
        left_camera_topic_arg,
        right_camera_topic_arg,
        display_output_arg,
        assessment_rate_arg,
        pose_assess_node
    ])

