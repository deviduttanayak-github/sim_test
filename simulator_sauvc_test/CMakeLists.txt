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
#sauvc_image_processing
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

#install(DIRECTORY include/sauvc_image_processing/
#  DESTINATION include/simultor_sauvc_test/
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(simulator_sauvc_test_lib include/simulator_sauvc_test/simulator_sauvc_test.h src/simulator_sauvc_test.cpp)

add_executable(simulator_sauvc_test include/simulator_sauvc_test/simulator_sauvc_test.h src/main.cpp include/simulator_sauvc_test/image_processor.h src/image_processor.cpp
)
target_link_libraries(simulator_sauvc_test simulator_sauvc_test_lib ${catkin_LIBRARIES}
 ${OpenCV_LIBS})


## added

add_library(ip_lib include/simulator_sauvc_test/image_processor.h src/image_processor.cpp)
#add_dependencies(ip_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ip_lib simulator_sauvc_test_lib ${catkin_LIBRARIES}
 ${OpenCV_LIBS})
