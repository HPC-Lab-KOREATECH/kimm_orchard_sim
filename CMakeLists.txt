cmake_minimum_required(VERSION 3.8)
project(kimm_orchard_sim)

find_package(ament_cmake REQUIRED)


install(DIRECTORY map config launch meshes models urdf worlds rviz scripts
    DESTINATION share/${PROJECT_NAME}/
)


install(PROGRAMS
    scripts/set_entity_state.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
