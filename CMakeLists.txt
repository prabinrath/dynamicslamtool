cmake_minimum_required(VERSION 2.8.3)
project(test_work1)
add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  genmsg
  sensor_msgs
  nav_msgs
  message_filters
  visualization_msgs
  tf
)

add_message_files(FILES pc_odom.msg)
generate_messages(DEPENDENCIES sensor_msgs nav_msgs)

#find_package(PCL 1.9 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

catkin_package(CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cluster_correspondence src/CloudCorrespondence.cpp include/CC/CloudCorrespondence.h include/CC/IncludeAll.h include/dtw/dtw.h)
target_link_libraries(cluster_correspondence ${catkin_LIBRARIES}) #${PCL_LIBRARIES}

add_executable(getpc src/getpc.cpp)
target_link_libraries(getpc ${catkin_LIBRARIES})

add_executable(cluster src/clustering_test.cpp)
target_link_libraries(cluster cluster_correspondence)

add_executable(pack src/make_packet.cpp)
target_link_libraries(pack ${catkin_LIBRARIES})
add_dependencies(pack test_work1_generate_messages_cpp)

add_executable(trans src/transform_test.cpp)
target_link_libraries(trans ${catkin_LIBRARIES})

add_executable(mattest src/trans_matrix_test.cpp)
target_link_libraries(mattest ${catkin_LIBRARIES})

add_executable(mov src/moving_object_test.cpp)
target_link_libraries(mov cluster_correspondence)

add_executable(keypoint src/keypoint_matching.cpp)
target_link_libraries(keypoint cluster_correspondence)

add_executable(chk src/check_corres.cpp include/dtw/dtw.h)
target_link_libraries(chk ${catkin_LIBRARIES})
