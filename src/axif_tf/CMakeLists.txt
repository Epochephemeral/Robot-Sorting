cmake_minimum_required(VERSION 2.8.3)
project(axif_tf)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    opencvtest
    dobot
    tf
    image_transport
    sensor_msgs
    geometry_msgs
    cv_bridge
    message_generation
)

add_message_files(
    FILES
    getPoint.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)
catkin_package(
   # INCLUDE_DIRS include
   # LIBRARIES axif_tf
    CATKIN_DEPENDS roscpp std_msgs opencvtest dobot tf image_transport sensor_msgs geometry_msgs cv_bridge message_runtime
   # DEPENDS system_lib
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
find_package(OpenCV)
    include_directories(${OpenCV_INCLUDE_DIRS})
    find_package(Eigen3 REQUIRED)
    include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${EIGEN3_LIBRARIES}
)

find_package(opencvtest REQUIRED)


add_executable(getmarker src/getmarker.cpp)
# target_link_libraries(getmarker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencvtest)
target_link_libraries(getmarker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(TFcamtorobot src/TFcamtorobot.cpp)
target_link_libraries(TFcamtorobot ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(transform_base src/transform_base.cpp)
target_link_libraries(transform_base ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})