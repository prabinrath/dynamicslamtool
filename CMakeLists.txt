cmake_minimum_required(VERSION 2.8.3)
project(test_work)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
)

#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cluster_correspondence src/CloudCorrespondence.cpp include/CC/CloudCorrespondence.h include/CC/IncludeAll.h include/dtw/dtw.h)
target_link_libraries(cluster_correspondence ${catkin_LIBRARIES})

add_executable(getpc src/getpc.cpp)
target_link_libraries(getpc ${catkin_LIBRARIES}) #${PCL_LIBRARIES}

add_executable(cluster src/clustering_test.cpp)
target_link_libraries(cluster cluster_correspondence)

add_executable(chk src/check_corres.cpp include/dtw/dtw.h)
target_link_libraries(chk ${catkin_LIBRARIES})
