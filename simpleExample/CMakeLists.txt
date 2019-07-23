cmake_minimum_required(VERSION 3.0)
# find YARP
find_package(YARP REQUIRED)
# set up our program
add_executable(simpleExample ./src/SimpleExample.h ./src/SimpleExample.cpp  ./src/main.cpp )
# link with YARP libraries
target_link_libraries(simpleExample ${YARP_LIBRARIES})
