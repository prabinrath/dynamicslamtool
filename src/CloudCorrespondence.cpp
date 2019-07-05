#include "CC/CloudCorrespondence.h"

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr CloudCorrespondence::tree = boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZI>>(new pcl::KdTreeFLANN<pcl::PointXYZI>);
extern ros::Publisher pub,marker_pub;

void CloudCorrespondence::filterPassThrough(float x,float y,float z)
{
	pcl::PassThrough<pcl::PointXYZI> pass;
  	pass.setInputCloud(cloud);
  	pass.setFilterFieldName("x");
  	pass.setFilterLimits(-x, x);
  	pass.filter(*cloud);
  	pass.setInputCloud(cloud);
  	pass.setFilterFieldName("y");
  	pass.setFilterLimits(-y, y);
  	pass.filter(*cloud);
  	pass.setInputCloud(cloud);
  	pass.setFilterFieldName("z");
  	pass.setFilterLimits(-0.5, z);
  	pass.filter(*cloud);
}

bool CloudCorrespondence::cluster_condition(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
{
	//very slow and useless
  	vector<int> pointIdxRadiusSearch;
  	vector<float> pointRadiusSquaredDistance;
  	if ( tree->radiusSearch(point_b, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  	{
  		if(pointIdxRadiusSearch.size()>5)
  			return true;
  	}
  	return false;
}

void CloudCorrespondence::computeClusters(float distance_threshold, string f_id)
{
	clusters.clear();
	cluster_indices.clear();
  centroid_collection.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
	
  	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  	ec.setClusterTolerance(distance_threshold);
  	ec.setMinClusterSize(200);
  	ec.setMaxClusterSize(35000);
  	ec.setInputCloud(cloud);
  	ec.extract(cluster_indices);
	/*
	pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec;
	tree->setInputCloud(cloud);
  	cec.setInputCloud(cloud);
  	cec.setConditionFunction(&CloudCorrespondence::cluster_condition);
  	cec.setClusterTolerance(distance_threshold);
  	cec.setMinClusterSize(400);
  	cec.setMaxClusterSize(25000);
  	cec.segment(cluster_indices);
	*/

  	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
	    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	    {
	    	cluster_collection->points.push_back(cloud->points[*pit]);
	    	cloud_cluster->points.push_back(cloud->points[*pit]);
	    }

	    cloud_cluster->header.frame_id = f_id;
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster);

	    Eigen::Vector4d temp;
	    pcl::compute3DCentroid(*cloud_cluster, temp);
	    pcl::PointXYZ centroid;
	    centroid.x = temp[0]; centroid.y = temp[1]; centroid.z = temp[2];
	    centroid_collection->points.push_back(centroid);
  	}

  	centroid_collection->width = centroid_collection->points.size();
	centroid_collection->height = 1;
	centroid_collection->is_dense = true;
  	cluster_collection->width = cluster_collection->points.size();
	cluster_collection->height = 1;
	cluster_collection->is_dense = true;
}

void CloudCorrespondenceMethods::calculateCorrespondenceCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr mp,double delta)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  	corr_est.setInputSource(fp);
  	corr_est.setInputTarget(fc);
	corr_est.determineReciprocalCorrespondences(*mp);
}

bool CloudCorrespondenceMethods::densityConstraint(const pcl::PointXYZI& point_a,pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree,long threshold)
{
  	vector<int> pointIdxRadiusSearch;
  	vector<float> pointRadiusSquaredDistance;
  	tree->setInputCloud(keypoints);
  	if ( tree->radiusSearch(point_a, 0.2, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  	{
  		if(pointIdxRadiusSearch.size()>threshold)
  			return true;
  	}
  	return false;
}

bool CloudCorrespondenceMethods::volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold)
{
	Eigen::Vector4f min;
  	Eigen::Vector4f max;
  	double volp,volc;

  	pcl::getMinMax3D(*fp, min, max);
  	volp = (max[0]-min[0])*(max[1]-min[1])*(max[2]-min[2]);
  	pcl::getMinMax3D(*fc, min, max);
  	volc = (max[0]-min[0])*(max[1]-min[1])*(max[2]-min[2]);

  	if((abs(volp-volc)/(volp+volc))<threshold)
  	{
  		return true;
  	}
  	return false;
}

vector<long> CloudCorrespondenceMethods::getClusterPointcloudChangeVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp,float resolution = 0.3f)
{
  cout<<"\n ********* PC Change *********** \n";
  vector<long> changed;
  srand((unsigned int)time(NULL));
  for(int j=0;j<mp->size();j++)
  {
    if(!volumeConstraint(c1[(*mp)[j].index_query],c2[(*mp)[j].index_match],0.5))
    {
      cout<<"Cluster Skipped!\n";
      changed.push_back(-1);
      continue;
    }

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree_cd(resolution);
    octree_cd.setInputCloud(c1[(*mp)[j].index_query]);
    octree_cd.addPointsFromInputCloud();
    octree_cd.switchBuffers();
    octree_cd.setInputCloud(c2[(*mp)[j].index_match]);
    octree_cd.addPointsFromInputCloud();
    
    vector<int> newPointIdxVector;
    octree_cd.getPointIndicesFromNewVoxels(newPointIdxVector);
    changed.push_back(newPointIdxVector.size());
  }
  return changed;
}

vector<double> CloudCorrespondenceMethods::getPointDistanceEstimateVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp)
{
  cout<<"\n ********* Distance Estimate *********** \n";
	vector<double> estimates;
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> corr_est;
  pcl::CorrespondencesPtr corrs;    
	for(int j=0;j<mp->size();j++)
	{
		if(!volumeConstraint(c1[(*mp)[j].index_query],c2[(*mp)[j].index_match],0.5))
		{
			cout<<"Cluster Skipped!\n";
			estimates.push_back(-1);
			continue;
		}

		corrs.reset(new pcl::Correspondences());
  	corr_est.setInputSource(c1[(*mp)[j].index_query]);
  	corr_est.setInputTarget(c2[(*mp)[j].index_match]);

		corr_est.determineCorrespondences(*corrs);
		double count = 0;
		for(int i=0;i<corrs->size();i++)
		{
			if((*corrs)[i].distance>0.005 && (*corrs)[i].distance<0.5)
			{
				count++;
			}
		}
		estimates.push_back(count/((c1[(*mp)[j].index_query]->points.size()+c2[(*mp)[j].index_match]->points.size())/2));
	}
	return estimates;
}

pcl::PointXYZ CloudCorrespondenceMethods::getDirectionVector(pcl::PointXYZI p1, pcl::PointXYZI p2)
{
  pcl::PointXYZ dir;
  float x,y,z;
  x = p2.x-p1.x;
  y = p2.y-p1.y;
  z = p2.z-p1.z;
  dir.x = x/sqrt(x*x+y*y+z*z);
  dir.y = y/sqrt(x*x+y*y+z*z);
  dir.z = z/sqrt(x*x+y*y+z*z);
  return dir;
}

vector<double> CloudCorrespondenceMethods::getDirectionCloudVariance(pcl::PointCloud<pcl::PointXYZI>::Ptr src,vector<pcl::PointIndices> &ci,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp)
{
    cout<<"\n ********* Direction Cloud Variance *********** \n";
    vector<double> directioncov;
    pcl::CorrespondencesPtr corrs(new pcl::Correspondences ());
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> corr_est;
    for(int j=0;j<mp->size();j++)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr upc(new pcl::PointCloud<pcl::PointXYZI>);
      for (vector<int>::const_iterator pit = ci[j].indices.begin(); pit != ci[j].indices.end(); ++pit)
      {
        upc->points.push_back(src->points[*pit]);
      }
      if(!volumeConstraint(upc,c2[(*mp)[j].index_match],0.5))
      {
        cout<<"Cluster Skipped!\n";
        continue;
      }

      corr_est.setInputSource(c1[(*mp)[j].index_query]);
      corr_est.setInputTarget(c2[(*mp)[j].index_match]);
      corr_est.determineCorrespondences(*corrs);

      pcl::PointCloud<pcl::PointXYZ> dir_cloud;
      for(int i=0;i<corrs->size();i++)
      {
        if((*corrs)[i].distance<0.01)
        {
          pcl::PointXYZ dir = getDirectionVector(upc->points[(*corrs)[i].index_query],c2[(*mp)[j].index_match]->points[(*corrs)[i].index_match]); 
          dir_cloud.points.push_back(dir);
        }
      }
      dir_cloud.width = dir_cloud.points.size();
      dir_cloud.height = 1;
      dir_cloud.is_dense = true;

      Eigen::Vector4f cp;
      pcl::compute3DCentroid(dir_cloud, cp);
      Eigen::Matrix3f covariance_matrix;
      pcl::computeCovarianceMatrix(dir_cloud, cp, covariance_matrix);

      cout<<covariance_matrix<<endl;

      sensor_msgs::PointCloud2 output;
      pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
      pcl::toPCLPointCloud2(dir_cloud,*cloud);
      pcl_conversions::fromPCL(*cloud, output);
      output.header.frame_id = "direction";
      pub.publish(output);

      sleep(1);

      directioncov.push_back(covariance_matrix(0,0)+covariance_matrix(1,1)+covariance_matrix(2,2));
    }
    return directioncov;
}

////////////////////////////////////////////////////////////////////Helping Methods

visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, int id, string f_id, string ns="bounding_box", float r=0.5, float g=0.5, float b=0.5)
{
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;

  pcl::compute3DCentroid(*cloud_cluster, centroid);
  pcl::getMinMax3D(*cloud_cluster, min, max);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = f_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = ns;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = (max[0]-min[0]);
  marker.scale.y = (max[1]-min[1]);
  marker.scale.z = (max[2]-min[2]);

  if (marker.scale.x ==0)
      marker.scale.x=0.1;

  if (marker.scale.y ==0)
    marker.scale.y=0.1;

  if (marker.scale.z ==0)
    marker.scale.z=0.1;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration(1);
  return marker;
}

visualization_msgs::Marker mark_direction(double x,double y,double z,int id, string f_id, string ns="direction_vector", float r=0.5, float g=0.5, float b=0.5)
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
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker.points.push_back(p);
  p.x = x;
  p.y = y;
  p.z = z;
  marker.points.push_back(p);

  marker.scale.x = 0.04;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.8;

  marker.lifetime = ros::Duration(1);
  return marker;
}