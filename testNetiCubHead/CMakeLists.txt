cmake_minimum_required(VERSION 3.5)
# find YARP
find_package(YARP REQUIRED)
# set up our program
add_executable(TestNetiCubHead ./src/test.h ./src/test.cpp  ./src/main.cpp )
# link with YARP libraries
target_link_libraries(TestNetiCubHead ${YARP_LIBRARIES})
