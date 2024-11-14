#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from os.path import join
import xacro
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Get bcr_bot package's share directory path
    # bcr_bot_path = get_package_share_directory('bcr_bot')
    kimm_orchard_sim_path = get_package_share_directory('kimm_orchard_sim')
    rviz_enabled = LaunchConfiguration('rviz', default='false')

    # DeclareLaunchArgument에 rviz 추가
    declare_rviz_enabled_argument = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Set to "true" to enable rviz'
    )

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_namespace = LaunchConfiguration("robot_namespace", default='')
    # world_file = LaunchConfiguration("world_file", default = join(kimm_orchard_sim_path, 'worlds', 'wall.sdf'))
    world_file = LaunchConfiguration("world_file", default = join(kimm_orchard_sim_path, 'worlds', 'apple_world.sdf'))
    #world_file = LaunchConfiguration("world_file", default = join(bcr_bot_path, 'worlds', 'small_warehouse.sdf'))

    # Path to the YAML file
    yaml_file_path = join(kimm_orchard_sim_path, 'config', 'sensor_params.yaml')

    with open(yaml_file_path, 'r') as file:
        yaml_content = file.read()
        
    # Path to the Xacro file
    xacro_path = join(kimm_orchard_sim_path, 'urdf', 'ranger_mini.xacro')
    doc = get_xacro_to_doc(xacro_path, {"sim_gazebo": "true"})
    map_xacro_path = join(kimm_orchard_sim_path, 'map', 'urdf', 'orchard_geometry.urdf')
    
    pcd_yaml_path = join(
        get_package_share_directory('pcd_cal'),
        'config',
        'tree_grid_sim.yaml'
    )

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': doc.toxml()},
                    {"yaml_content": yaml_content}]
        # remappings=[
        #     ('/joint_states', PythonExpression(['"', robot_namespace, '/joint_states"'])),
        # ]
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', "/robot_description",
            '-entity', PythonExpression(['"', robot_namespace, '_robot"']), #default enitity name _bcr_bot
            '-z', "0.5",
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )

    spawn_map = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', map_xacro_path,
            '-entity', PythonExpression(['"', robot_namespace, 'orchard"']), #default enitity name _bcr_bot
            '-z', "0.15",
            '-x', "5.05",
            '-y', "-15.2",
            '-Y', orientation_yaw
        ]
    )

    set_entity_state = Node(
        package='kimm_orchard_sim',
        executable='set_entity_state.py'
    )

    goal_fl=Node(
            package='kimm_orchard_sim', 
            executable='goal_flag'
        )


    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    # minwoo's code 
    
    forward_position_controller = ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller']
        )

    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller']
        )

    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster']
        )
    
    rviz = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d', join(kimm_orchard_sim_path, 'rviz', 'entire_setup.rviz')],
        condition=IfCondition(rviz_enabled)

	)
    
    controller_teleop = Node(
        package='kimm_orchard_sim',
        executable='ranger_controller_key.py',
        name='commander',
    )

    utm_pub = Node(
        package='kimm_orchard_sim',
        executable='utm_publisher.py',
        name='utm',
        parameters=[{"use_sim_time": use_sim_time}],
    )

    path_pub = Node(
        package='kimm_orchard_sim',
        executable='path_publisher.py',
        name='path',
        parameters=[{"use_sim_time": use_sim_time}],
    )

    local_path_pub = Node(
        package='kimm_orchard_sim',
        executable='local_path_publisher.py',
        name='path',
        parameters=[{"use_sim_time": use_sim_time}],
    )
    

    state_pub = Node(
        package='kimm_orchard_sim',
        executable='pub_states.py',
        name='pub_states',
        parameters=[{"use_sim_time": use_sim_time}],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(['"', camera_enabled, '" == "true"'])),
    )

    base_link_state_pub = Node(
        package='kimm_orchard_sim',
        executable='base_link_state.py',
        name='base_link_state',
        output='screen'
    )
    
    laser_scan = Node(
        package='pcd_cal',
        executable='laser_scan',
        name='laser_scan',
        output='screen',
        parameters=[pcd_yaml_path]
    )

     # tf2_ros static_transform_publisher 실행
    static_transform_publisher = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0.4', '0', '0', '0', 'ouster', 'laser_data_frame'],
        output='screen'
    )


    # kimm_orchard_sim mappub 실행
    map_pub = Node(
        package='kimm_orchard_sim',
        executable='mappub.py',
        name='mappub',
        parameters=[{"use_sim_time": use_sim_time}],
        output='screen',
    )

    
    # Include lio_sam run_loc.launch.py
    lio_sam_launch_file_path = join(
        get_package_share_directory('lio_sam'), # Replace 'lio_sam' with the correct package name if different
        'launch',
        'run_loc.launch.py'
    )
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lio_sam_launch_file_path)
        # Here you can add launch arguments if 'run_loc.launch.py' accepts any
    )

    # Include nav2_bringup navigation_launch.py with use_sim_time:=True
    nav2_bringup_launch_file_path = join(
        get_package_share_directory('nav2_bringup'), # Replace 'nav2_bringup' with the correct package name if different
        'launch',
        'navigation_launch.py'
    )
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file_path),
        launch_arguments={'use_sim_time': 'True'}.items() # Passing the 'use_sim_time' argument as True
    )

    state_machine = Node(
        package='kimm_orchard_sim',
        executable='state_machine',
        name='mappub',
        parameters=[{"use_sim_time": use_sim_time}],
        output='screen',
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('world', default_value = world_file),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0"),
        DeclareLaunchArgument("position_y", default_value="0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),
        # DeclareLaunchArgument('robot_description', default_value=doc.toxml()),

        # DeclareLaunchArgument('controller_configuration', default_value=controller_param_path),
        declare_rviz_enabled_argument,
        gazebo,
        robot_state_publisher,
        spawn_map,
        spawn_entity,
        forward_position_controller,
        forward_velocity_controller,
        # goal_fl,
        # rqt_robot_steering,
        rviz,
        set_entity_state,
        controller_teleop,
        utm_pub,
        path_pub,
        state_pub,
        joint_state_broadcaster,
        joint_state_publisher_gui,
        joint_state_publisher,
        base_link_state_pub,
        laser_scan,
        static_transform_publisher,
        map_pub,
        local_path_pub,
        # lio_sam_launch,
        # nav2_bringup_launch
        state_machine
    ])
    
