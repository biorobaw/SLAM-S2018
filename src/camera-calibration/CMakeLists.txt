cmake_minimum_required(VERSION 3.10)
project(camera_calibration)

set(CMAKE_CXX_STANDARD 11)

add_executable(camera_calibration main.cpp)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_calibration ${OpenCV_LIBS})