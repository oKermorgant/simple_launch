cmake_minimum_required(VERSION 3.5)
project(simple_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY example
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/call_service scripts/set_parameters scripts/generate_gz_world
        DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
