# Copyright: 2010 RobotCub Consortium
# Author: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

cmake_minimum_required(VERSION 3.5)

project(TestGetPidApp)

find_package(YARP 3.5.1 REQUIRED COMPONENTS os dev)

set(folder_header src/TestGetPidApp.h)

set(folder_source src/TestGetPidApp.cpp
                  src/main.cpp)

include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(${PROJECT_NAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

