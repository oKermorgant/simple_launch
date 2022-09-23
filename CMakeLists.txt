cmake_minimum_required(VERSION 3.5)
project(simple_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        example
        DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
