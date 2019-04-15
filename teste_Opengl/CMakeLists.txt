cmake_minimum_required(VERSION 2.8)

project(teste_Opengl)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)

add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME} OpenGL::OpenGL ${GLUT_LIBRARY} )
