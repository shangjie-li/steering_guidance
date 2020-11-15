cmake_minimum_required(VERSION 2.8.3)
project(steering_guidance)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/steering_guidance_node.cpp src/steering_guidance_core.cpp)

target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


