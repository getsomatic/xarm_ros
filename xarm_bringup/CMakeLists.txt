cmake_minimum_required(VERSION 3.5)
project(xarm_bringup)

add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

ament_package()