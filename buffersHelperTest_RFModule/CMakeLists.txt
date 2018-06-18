cmake_minimum_required(VERSION 3.0)
# find YARP
find_package(YARP REQUIRED)
# set up our program
add_executable(buffTest ./src/BuffersHelper.h ./src/buffHelpTest.h ./src/buffHelpTest.cpp ./src/main.cpp )
# link with YARP libraries
target_link_libraries(buffTest ${YARP_LIBRARIES})
