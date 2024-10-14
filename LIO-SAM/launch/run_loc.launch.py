import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    parameter_file_for_robot_localzation = LaunchConfiguration('params_file_for_robot_localzation')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params_of_localization_mode.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    params_declare2 = DeclareLaunchArgument(
        'params_file_for_robot_localzation',
        default_value=os.path.join(
            share_dir, 'config', 'robot_localization_of_localization_mode_param.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        params_declare2,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.3 0.0 0.0 0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
            ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments='0.0 0.0 0.0 0.0 0.0 0 base_link gps'.split(' '),
        #     parameters=[parameter_file],
        #     output='screen'
        #     ),
        # Node(
        #     package='lio_sam',
        #     executable='lio_sam_simpleGpsOdom',
        #     name='lio_sam_simpleGpsOdom',
        #     parameters=[parameter_file],
        #     output='screen'
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_tf_static',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_localization',
            name='lio_sam_localization',
            parameters=[parameter_file],
            output='screen'
        ),
        # Node(
        #     package='lio_sam',
        #     executable='lio_sam_gps_converter',
        #     name='lio_sam_gps_converter',
        #     parameters=[parameter_file],
        #     output='screen'
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_gps',
            parameters=[parameter_file_for_robot_localzation],
            respawn=True,
            remappings=[
                ('odometry/filtered', 'odometry/navsat'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat',
            parameters=[parameter_file_for_robot_localzation],
            respawn=True,
            remappings=[
                ('imu', 'imu_hpc'),
                ('gps/fix', 'fix'),
                ('odometry/filtered', 'odometry/navsat'),
            ],
        )
    ])
