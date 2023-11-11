cmake_minimum_required(VERSION 3.0.2)
project(lane_detection
  VERSION 1.0.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  # xycar_msgs
)

find_package(OpenCV 4.5.5 REQUIRED PATHS /root/3rd/opencv/install/lib/cmake/opencv4)

catkin_package()

add_subdirectory(modules)

if(CMAKE_BUILD_TYPE MATCHES Debug)
   add_definitions(-DDEBUG=1)
endif()

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)