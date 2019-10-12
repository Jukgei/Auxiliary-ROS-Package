cmake_minimum_required(VERSION 2.8.3)
project(auxiliary)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)


add_compile_options(-std=c++11)
SET(SRC_LIST src/main.cpp src/datapackage.cpp src/arm.cpp src/auxiliary.cpp)
## Declare ROS messages and services
add_message_files(FILES state.msg controls.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/local/include/serial)

#add_executable(auxiliary src/main.cpp src/datapackage.cpp src/arm.cpp)
add_executable(auxiliary ${SRC_LIST})
target_link_libraries(auxiliary ${catkin_LIBRARIES} serial)
#add_dependencies(auxiliary beginner_tutorials_generate_messages_cpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)
