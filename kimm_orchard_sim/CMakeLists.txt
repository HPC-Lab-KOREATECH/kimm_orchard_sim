cmake_minimum_required(VERSION 3.8)
project(kimm_orchard_sim)

find_package(ament_cmake REQUIRED)


install(DIRECTORY map config launch meshes models urdf worlds rviz scripts
    DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/set_entity_state.py
    scripts/ranger_controller.py
    scripts/keyboard_teleop.py
    scripts/utm_publisher.py
    scripts/pub_states.py
    scripts/path_debug/path_publisher.py
    scripts/base_link_state.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
