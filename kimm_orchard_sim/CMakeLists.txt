cmake_minimum_required(VERSION 3.8)
project(kimm_orchard_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(goal_flag scripts/goal_pose_flag.cpp)

ament_target_dependencies(goal_flag rclcpp std_msgs nav_msgs)

install(TARGETS
  goal_flag
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY map config launch meshes models urdf worlds rviz scripts
    DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/set_entity_state.py
    scripts/ranger_controller.py
    scripts/ranger_controller_key.py
    scripts/keyboard_teleop.py
    scripts/utm_publisher.py
    scripts/pub_states.py
    scripts/path_debug/path_publisher.py
    scripts/base_link_state.py
    scripts/mappub.py
    scripts/path_debug/local_path_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
