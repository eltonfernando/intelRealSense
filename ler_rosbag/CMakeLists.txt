cmake_minimum_required(VERSION 2.8)

project(ler_rosbag)
find_package(realsense2 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(PCL 1.3 REQUIRED)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
