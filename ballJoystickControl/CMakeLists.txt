#
# Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Author: Valentina Gaggero valentina.gaggero@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#


cmake_minimum_required(VERSION 3.0)
# find YARP
find_package(YARP REQUIRED)

# set up our program
add_executable(ballJoystickControl ./src/ballJoystickControl.h ./src/ballJoystickControl.cpp  ./src/joystick.h ./src/main.cpp )
#add_executable(ballJoystickControl joystick.h ballJoystickControl.h ballJoystickControl.cpp main.cpp )
# link with YARP libraries

target_link_libraries(ballJoystickControl ${YARP_LIBRARIES})

#set(PROJECTNAME BallJoystickControl)
#project(${PROJECTNAME})
#
#file(GLOB folder_source *.cpp)
#file(GLOB folder_header *.h)
#
#source_group("Source Files" FILES ${folder_source})
#source_group("Header Files" FILES ${folder_header})
#
#include_directories(${GSL_INCLUDE_DIRS}
#                    ${YARP_INCLUDE_DIRS}
#                    ${ICUB_INCLUDE_DIRS})
#
#add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
#
#target_link_libraries(${PROJECTNAME} ctrlLib ${GSL_LIBRARIES} ${YARP_LIBRARIES})
#
#install(TARGETS ${PROJECTNAME} DESTINATION bin)
#
