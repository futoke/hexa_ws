from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    odometry_node = Node(
        package="ros2_laser_scan_matcher",
        parameters=[{
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            # 'map_frame': 'map',
            'laser_frame': 'base_laser',
            'publish_odom': '/odom',
            'publish_tf': True
        }],
        executable="laser_scan_matcher",
        name="odometry_publisher",
    )

    # Create and return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description="Flag to enable use_sim_time"
        ),
        odometry_node
    ])
