cmake_minimum_required(VERSION 3.0)
# find YARP
find_package(YARP REQUIRED)
find_package( OpenCV REQUIRED )
# set up our program
set(ASSISTIVE_REHAB_INC         $ENV{ROBOT_CODE}/assistive-rehab/lib/include)
set(YARP_CV                     $ENV{ROBOT_CODE}/yarp/src/libYARP_cv/include)
set(ASSISTIVE_REHAB_SRC         $ENV{ROBOT_CODE}/assistive-rehab/lib/src/dtw.cpp
                                $ENV{ROBOT_CODE}/assistive-rehab/lib/src/helpers.cpp
                                $ENV{ROBOT_CODE}/assistive-rehab/lib/src/skeleton.cpp)
include_directories (${ASSISTIVE_REHAB_INC} ${YARP_CV})

add_executable(follower ./src/follower.h ./src/TargetRetriver.h ./src/Ball3DPointRetriver.cpp ./src/Person3DRetriver.cpp ./src/simFramePainter.h ./src/simFramePainter.cpp ./src/Person3DRetriver.h ./src/follower.cpp ./src/main.cpp  ${ASSISTIVE_REHAB_SRC})
# link with YARP libraries
target_link_libraries(follower ${YARP_LIBRARIES} ${OpenCV_LIBS} )
