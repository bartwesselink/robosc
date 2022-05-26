cmake_minimum_required(VERSION 3.5)
project(controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)


add_library(controller_engine include/controller/controller_engine.c)
add_executable(interface src/member_function.cpp)

add_definitions(-DMAX_NUM_EVENTS=0 -DEVENT_OUTPUT=1)

target_link_libraries(interface controller_engine)

ament_target_dependencies(interface rclcpp std_msgs rclcpp_action geometry_msgs nav2_msgs)

install(TARGETS
        interface
        DESTINATION lib/${PROJECT_NAME})

ament_package()
