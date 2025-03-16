# Copyright 2024 Walter Lucetti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    
    node_name = LaunchConfiguration('node_name')

    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr_slam.yaml'
    )

    # SLAM Toolbox configuration for LDLidar
    slam_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'slam_toolbox.yaml'
    )

    # EKF configuration
    ekf_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'ekf.yaml'
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            # YAML files
            lc_mgr_config_path  # Parameters
        ]
    )

    # lc_mgr_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_navigation',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': False,
    #         'autostart': True,
    #         'node_names': ['map_server', 'amcl', 'planner_server',
    #                        'controller_server', 'recoveries_server', 'bt_navigator',
    #                        'waypoint_follower']
    #     }]
    # )

    # SLAM Toolbox node in async mode
    slam_toolbox_node = LifecycleNode(
          package='slam_toolbox',
          executable='async_slam_toolbox_node',
          namespace='',
          name='slam_toolbox',
          output='screen',
          parameters=[
            # YAML files
            slam_config_path # Parameters
          ],
          remappings=[
              ('/scan', '/ldlidar_node/scan')
          ]          
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': '/home/spider/hexa_ws/map-1.yaml'}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        parameters=[{'use_sim_time': False,
                     'alpha1': 0.2,
                     'alpha2': 0.2,
                     'alpha3': 0.2,
                     'alpha4': 0.2}]
    )

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
             'autostart': 'true',
            'map': '/home/spider/hexa_ws/map-1.yaml'
        }.items()
    )

    # Узел локализации
    loc_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
          # YAML files
          ekf_config_path # Parameters
        ]
    )

    # Узел кинематики
    tripod_gait_node = Node(
        package='hexa_ik',
        executable='main_gait',
        output='screen'
    )

    # Узел для джойстика (PS4)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        # parameters=[{'device': '/dev/input/js0'}]  # Убедись, что указываешь правильное устройство
    )

    # Узел серво драйвера
    servo_node = Node(
        package='hexa_servo',
        executable='servo',
        name='hexa_servo',
        output='screen'
    )

    # Узел IMU
    imu_node = Node(
        package='mpu6050_driver',
        executable='mpu6050_driver',
        name='mpu6050_driver',
        output='screen'
    )

    # Include LDLidar launch
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': 'ldlidar_node'
        }.items()
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ]
    )

    # Fake odom publisher
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        # arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link'
        ]
    )

    # RVIZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'config',
        'ldlidar_slam.rviz'
    )

    # RVIZ2node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    ld.add_action(tripod_gait_node)
    ld.add_action(joy_node)
    ld.add_action(servo_node)
    ld.add_action(imu_node)
    ld.add_action(loc_node)
    # ld.add_action(map_server)
    # ld.add_action(amcl_node)
    # ld.add_action(navigation_launch)
    ld.add_action(static_tf_map_odom)


    # Launch Nav2 Lifecycle Manager
    ld.add_action(lc_mgr_node)

    # Launch SLAM Toolbox node
    ld.add_action(slam_toolbox_node)

    # Launch fake odom publisher node
    ld.add_action(fake_odom)

    # Call LDLidar launch
    ld.add_action(ldlidar_launch)

    # Start RVIZ2
    ld.add_action(rviz2_node)
    

    return ld
