cmake_minimum_required(VERSION 2.8.3)
project(test_work2)
add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  visualization_msgs
  tf
)

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cluster_correspondence2 src/CloudCorrespondence.cpp include/CC/CloudCorrespondence.h include/CC/IncludeAll.h include/dtw/dtw.h)
target_link_libraries(cluster_correspondence2 ${catkin_LIBRARIES}) #${PCL_LIBRARIES}

add_executable(mov src/moving_object_test.cpp)
target_link_libraries(mov cluster_correspondence2)
