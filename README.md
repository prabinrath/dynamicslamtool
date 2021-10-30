# Moving Object Removal from 3D Pointcloud

MOR is a ROS package that takes pointcloud and external odometry data as input to run detection and removal of moving clusters of points form the cloud in real-time. It based on the basic idea of relative motion, where the knowledge of our own motion with respect to ground helps determining the motion of other objects with respect to ground.

The incoming point clouds and odometry data are time synchronized to make pairs such that clouds are matched with their pose as captured by the inertial sensors. The transformation matrix obtained from the pose difference between two consecutive data samples is used to transform the historical pointcloud to the pose of the latest pointcloud. The transformed historical cloud and the actual sampled cloud are analyzed to detect the moving clusters in the data which are further tracked, removed and published as filtered cloud. 

This package also implements a new ground plane removal method based on voxel covariance and binning of points along the normal axis of the plane. 

## Installation

The package depends on PCL 1.8, standard libraries and roscpp.
Copy the package to the workspace and build it.

## Usage

```c++
void dataSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
   /*time synchronized data packet received*/
   pcl::PCLPointCloud2 cloud;
   pcl_conversions::toPCL(*input, cloud); //convert data into required format

   mor->pushRawCloudAndPose(cloud,odm->pose.pose); //mor is the pointer to the MOR class object
   if(mor->filterCloud(cloud,"/filtered")) //second argument is the frame id for the published filtered pointcloud
   {
      pub.publish(mor->output);
   }
}
```
