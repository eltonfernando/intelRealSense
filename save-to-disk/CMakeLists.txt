# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesSaveToDisk)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
find_package(realsense2 REQUIRED)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

# save-to-disk
add_executable(rs-save-to-disk rs-save-to-disk.cpp)
target_link_libraries(rs-save-to-disk ${realsense2_LIBRARY})
#target_link_libraries(rs-save-to-disk ${DEPENDENCIES})
include_directories(/home/elton/librealsense/examples/rs-save-to-disk ../../common ../../third-party ../../third-party/tclap/include)
set_target_properties (rs-save-to-disk PROPERTIES FOLDER "Examples")
#install(TARGETS rs-save-to-disk RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
