#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <unistd.h>
#include <ctime>
#include <unordered_map>
#include <string>

ros::Publisher pub,marker_pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr ca, cb;

visualization_msgs::Marker mark_direction(pcl::PointXYZI pt,double x,double y,double z,int id, std::string f_id, std::string ns="direction_vector", float r=0.5, float g=0.5, float b=0.5)
{
  uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
  visualization_msgs::Marker marker;
  marker.header.frame_id = f_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = ns;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.x = 1;
  marker.pose.orientation.y = 1;
  marker.pose.orientation.z = 1;
  marker.pose.orientation.w = 1;

  geometry_msgs::Point p;
  p.z = pt.x;
  p.x = pt.y;
  p.y = pt.z;
  marker.points.push_back(p);
  p.z = pt.x+x;
  p.x = pt.y+y;
  p.y = pt.z+z;
  marker.points.push_back(p);

  marker.scale.x = 0.04;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.8;

  marker.lifetime = ros::Duration(1);
  return marker;
}

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

void statistical_outlier_removal(const sensor_msgs::PointCloud2ConstPtr& input)
{
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2,cloud_filtered;
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *ca);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(ca);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.8);
    sor.filter(*cb);  
    
    pcl::toPCLPointCloud2(*cb,*cloud);
    pcl_conversions::fromPCL(*cloud, output);
    output.header.frame_id = "/test";
    pub.publish(output);
}

void groundPlaneRemoval(const sensor_msgs::PointCloud2ConstPtr &input)
{
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *ca);
    cb.reset(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr dsc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(ca);
    vg.setLeafSize(0.1,0.1,0.1);
    vg.filter(*dsc);

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    tree->setInputCloud(ca);

    std::vector<std::vector<int>> index_bank;
    for(int i=0;i<dsc->points.size();i++)
    {
      std::vector<int> ind;
      std::vector<float> dist;
      if(tree->radiusSearch(dsc->points[i], 0.1, ind, dist) > 0 )
      //if(tree->nearestKSearch(dsc->points[i], 20, ind, dist) > 0 )
      {
        if(ind.size()>3)
        {
          pcl::PointCloud<pcl::PointXYZI> temp;
          for(int j=0;j<ind.size();j++)
          {
            temp.points.push_back(ca->points[ind[j]]);
          }
          temp.width = temp.points.size();
          temp.height = 1;

          Eigen::Vector4f cp;
          pcl::compute3DCentroid(temp, cp);
          Eigen::Matrix3f covariance_matrix;
          pcl::computeCovarianceMatrix(temp, cp, covariance_matrix);
          if(fabs(covariance_matrix(0,2))<0.001 && fabs(covariance_matrix(1,2))<0.001 && fabs(covariance_matrix(2,2))<0.001)
          {
            cb->points.push_back(dsc->points[i]);
            index_bank.push_back(ind);
          }
        }
      }
    }

    std::unordered_map<float,std::vector<int>> bins;
    for(int i=0;i<cb->points.size();i++)
    {
      float key = (float)((int)(cb->points[i].z*10))/10;
      bins[key].push_back(i);
    }
    cb.reset(new pcl::PointCloud<pcl::PointXYZI>);
    float tracked_key = bins.begin()->first;
    int mode = bins.begin()->second.size();
    for(std::unordered_map<float,std::vector<int>>::iterator it=bins.begin();it!=bins.end();it++)
    {
      if(it->second.size()>mode)
      {
        mode = it->second.size();
        tracked_key = it->first;
      }
    }
    pcl::PointIndicesPtr ground_plane(new pcl::PointIndices);
    for(int i=0;i<bins[tracked_key].size();i++)
    {
      for(int j=0;j<index_bank[bins[tracked_key][i]].size();j++)
      {
        ground_plane->indices.push_back(index_bank[bins[tracked_key][i]][j]);
      }
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(ca);
    extract.setIndices(ground_plane);
    extract.setNegative(false);
    extract.filter(*cb);

    cb->width = cb->points.size();
    cb->height = 1;
    pcl::toPCLPointCloud2(*cb,*cloud);
    pcl_conversions::fromPCL(*cloud, output);
    output.header.frame_id = "/current";
    pub.publish(output);

    pcl::toPCLPointCloud2(*ca,*cloud);
    pcl_conversions::fromPCL(*cloud, output);
    output.header.frame_id = "/previous";
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
    //seg.setOptimizeCoefficients(false);
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
  output.header.frame_id = "/current";
  pub.publish(output);
}

void cropbox_check(const sensor_msgs::PointCloud2ConstPtr &input)
{
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *ca);

    pcl::CropBox<pcl::PointXYZI> cropBoxFilter (true);
    cropBoxFilter.setInputCloud(ca);
    Eigen::Vector4f min_pt(-1.0f, -1.0f, -1.0f, 1.0f);
    Eigen::Vector4f max_pt(1.0f, 1.0f, 1.0f, 1.0f);
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);
    cropBoxFilter.setNegative(true);
    cropBoxFilter.setTranslation(Eigen::Vector3f(1, 1, 0));
    cropBoxFilter.filter(*cb);

    pcl::toPCLPointCloud2(*cb,*cloud);
    pcl_conversions::fromPCL(*cloud, output);
    output.header.frame_id = "/current";
    pub.publish(output);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cropbox_check);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("bbox", 10);

  ca.reset(new pcl::PointCloud<pcl::PointXYZI>);
  cb.reset(new pcl::PointCloud<pcl::PointXYZI>);
  ca->height = -1; cb->height = -1;

  ros::spin();
}
