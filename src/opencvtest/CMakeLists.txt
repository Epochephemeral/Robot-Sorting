cmake_minimum_required(VERSION 2.8.3)
project(opencvtest)



## Compile as C++11, supported in ROS Kinetic and newer

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation #自定义消息类型依赖
)


add_message_files(
  FILES
  pixel_point0.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  #自定义消息依赖
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencvtest
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS system_lib
)


include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV 3)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(sorting src/sorting.cpp)
target_link_libraries(sorting ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(opencvtest src/sorting.cpp)

target_link_libraries(sorting ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencvtest)

add_executable(hsv src/hsv.cpp)
target_link_libraries(hsv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(savepic src/savepic.cpp)
target_link_libraries(savepic ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})