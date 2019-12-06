cmake_minimum_required(VERSION 2.8.3)
project(ardrone_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
    drone_custom
)



#add_message_files(
#  FILES
#  drone_custom/msg/DadosVisao.msg
#)





catkin_package( 
    INCLUDE_DIRS include
    LIBRARIES ardrone_controller_lib
    CATKIN_DEPENDS  roscpp rospy std_msgs sensor_msgs message_runtime
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS},
    src
    include/ardrone_controller_lib
)

include_directories(${CMAKE_CURRENT_LIST_DIR}/cmake/Modules)

add_library(ardrone_controller_lib
    src/pid.cpp
)



target_link_libraries(ardrone_controller_lib ${catkin_LIBRARIES})
add_dependencies(ardrone_controller_lib ${catkin_EXPORTED_TARGETS})

add_executable(ardrone_controller src/controller.cpp)
target_link_libraries(ardrone_controller ardrone_controller_lib ${catkin_LIBRARIES})

#add_executable(manip_node src/manip_node.cpp)
#target_link_libraries(manip_node rexrov_essentials_LIBRARIES ${catkin_LIBRARIES})