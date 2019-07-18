cmake_minimum_required(VERSION 2.8.3)
project(test_work2)
add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  visualization_msgs
  tf
)

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mod_lib src/MovingObjectRemoval.cpp include/MOR/MovingObjectRemoval.h include/MOR/IncludeAll.h)
target_link_libraries(mod_lib ${catkin_LIBRARIES})

add_executable(mov_i src/internal_sync_test.cpp)
target_link_libraries(mov_i mod_lib)

add_executable(mov_e src/external_sync_test.cpp)
target_link_libraries(mov_e mod_lib)
