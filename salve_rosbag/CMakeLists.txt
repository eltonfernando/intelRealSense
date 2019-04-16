cmake_minimum_required(VERSION 2.8)

project(salve_rosbag)
find_package(realsense2 REQUIRED)
find_package( OpenCV REQUIRED )

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
