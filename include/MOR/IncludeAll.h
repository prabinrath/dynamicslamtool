/*Includes all the required headers for the package*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/correspondence.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <flann/flann.h>
#include <iostream>
#include <unordered_map>
#include <string>
#include <fstream>
#include <ctime>

//#define VISUALIZE 
/*This defination flag helps to visualize the moving cluster and individual clusters detected
by the algorithm*/

//#define INTERNAL_SYNC
/*This defination flag turns on the internal message synchronizer to subscribe to the topics given
by the config file. If the VISUALIZE flag is defined, it publishes the filtered pointcloud after 
removing the moving objects along with the detection results, on topics defined in config file.*/