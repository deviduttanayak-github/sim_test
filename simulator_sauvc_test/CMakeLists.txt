cmake_minimum_required(VERSION 2.8.3)
project(simulator_sauvc_test)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
hammerhead
roscpp
roslib
std_msgs
sensor_msgs
image_transport
genmsg
message_filters
message_generation
cv_bridge
pid_controller
vectornav
)

add_service_files(
  FILES
  Coordinates.srv

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(OpenCV 3.2 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(simulator_sauvc_test_lib include/simulator_sauvc_test/simulator_sauvc_test.h src/simulator_sauvc_test.cpp)

add_executable(simulator_sauvc_test include/simulator_sauvc_test/simulator_sauvc_test.h src/main.cpp)
target_link_libraries(simulator_sauvc_test simulator_sauvc_test_lib ${catkin_LIBRARIES})
