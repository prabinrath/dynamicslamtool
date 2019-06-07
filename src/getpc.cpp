#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <unistd.h>
#include <ctime>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr ca, cb;

void detect_change(const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*input, *cloud);	//convert to PCLPointCloud2 type - a standard for dealing with ROS pointCloud2 type
  
  if(ca->height==-1)
  {
  	pcl::fromPCLPointCloud2(*cloud, *ca);	//convert to any PointT pointcloud type - needs tobe done for applying user specific PCL operations
  	std::cout<<"added ca\n";
  }
  else if(cb->height==-1)
  {
  	pcl::fromPCLPointCloud2(*cloud, *cb);
  	std::cout<<"added cb\n";
  }
  else
  {
  	srand ((unsigned int) time (NULL));
  	float resolution = 0.1f;
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree (resolution);
	octree.setInputCloud(ca);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();
	octree.setInputCloud(cb);
	octree.addPointsFromInputCloud();
	
	std::vector<int> newPointIdxVector;
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);
	std::cout<<newPointIdxVector.size();
	std::vector< pcl::PointXYZI,Eigen::aligned_allocator< pcl::PointXYZI > > vec;
	for (int i = 0; i < newPointIdxVector.size(); i++)
	{
		vec.push_back(cb->points[newPointIdxVector[i]]);
	}
	pcl::PointCloud<pcl::PointXYZI> change;
	change.points = vec;
	change.width = vec.size();
	change.height = 1;
	
	pcl::fromPCLPointCloud2(*cloud, *ca);
	pcl::toPCLPointCloud2(change,*cloud);
	pcl_conversions::fromPCL(*cloud, output);
	output.header.frame_id = "/test";
	pub.publish(output);
	std::cout<<"published diff\n";
	cb->height = -1;
  }
}

void voxel_filter(const sensor_msgs::PointCloud2ConstPtr& input)
{
	sensor_msgs::PointCloud2 output;
  	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2,cloud_filtered;
  	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  	pcl_conversions::toPCL(*input, *cloud);
  	
  	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  	sor.setInputCloud(cloudPtr); //takes ConstPtr as argument
  	sor.setLeafSize (0.05f, 0.05f, 0.05f);
  	sor.filter(cloud_filtered);
  	
	pcl_conversions::fromPCL(cloud_filtered, output);
	output.header.frame_id = "/test";
	pub.publish(output);
}

void pass_through(const sensor_msgs::PointCloud2ConstPtr &input)
{
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *ca);
    
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(ca);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.5, 5.0);
    pass.filter(*cb);

    *ca = *cb;
    pass.setInputCloud(ca);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-3.0, 3.0);
    pass.filter(*cb);

    *ca = *cb;
    pass.setInputCloud(ca);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-3.0, 3.0);
    pass.filter(*cb);
    
    pcl::toPCLPointCloud2(*cb,*cloud);
    pcl_conversions::fromPCL(*cloud, output);
    output.header.frame_id = "/test";
    pub.publish(output);
}

void ransac_check(const sensor_msgs::PointCloud2ConstPtr &input)
{
  sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *ca);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(ca);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return;
    }
    
    std::vector< pcl::PointXYZI,Eigen::aligned_allocator< pcl::PointXYZI > > vec;
  for (int i = 0; i < inliers->indices.size (); i++)
  {
    vec.push_back(ca->points[inliers->indices[i]]);
  }
  
  cb->points = vec;
  cb->width = vec.size();
  cb->height = 1;
  
  pcl::toPCLPointCloud2(*cb,*cloud);
  pcl_conversions::fromPCL(*cloud, output);
  output.header.frame_id = "/test";
  pub.publish(output);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, voxel_filter);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  ca.reset(new pcl::PointCloud<pcl::PointXYZI>);
  cb.reset(new pcl::PointCloud<pcl::PointXYZI>);
  ca->height = -1; cb->height = -1;
  ros::spin();
}
