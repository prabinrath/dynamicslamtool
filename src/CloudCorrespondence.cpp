#include "CC/CloudCorrespondence.h"

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr CloudCorrespondence::tree = boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZI>>(new pcl::KdTreeFLANN<pcl::PointXYZI>);

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
  	std::vector<int> pointIdxRadiusSearch;
  	std::vector<float> pointRadiusSquaredDistance;
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

  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
	    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	    {
	    	cluster_collection->points.push_back(cloud->points[*pit]);
	    	cloud_cluster->points.push_back(cloud->points[*pit]);
	    }

	    cloud_cluster->header.frame_id = f_id;
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	    sor.setInputCloud(cloud_cluster);
	    sor.setMeanK(100);
	    sor.setStddevMulThresh(0.8);
	    sor.filter(*cloud_cluster);

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

void CloudCorrespondenceMethods::calculateCorrespondenceCentroidKdtree(pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr mp,double delta)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
	mp.reset(new pcl::Correspondences());
  	corr_est.setInputSource(fp);
  	corr_est.setInputTarget(fc);
	corr_est.determineReciprocalCorrespondences(*mp);
}

vector<double> CloudCorrespondenceMethods::getPointDistanceEstimateVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp)
{
	vector<double> estimates;
  	pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> corr_est;
  	pcl::CorrespondencesPtr corrs;

	for(int j=0;j<mp->size();j++)
	{
		corrs.reset(new pcl::Correspondences());
  		corr_est.setInputSource(c1[(*mp)[j].index_query]);
  		corr_est.setInputTarget(c2[(*mp)[j].index_match]);
		corr_est.determineCorrespondences(*corrs);
		double count = 0;
		for(int i=0;i<corrs->size();i++)
		{
			if((*corrs)[i].distance>0.01 /*&& (*corrs)[i].distance<0.0001*/)
			{
				count++;
			}
		}
		estimates.push_back(count/((c1[(*mp)[j].index_query]->points.size()+c2[(*mp)[j].index_match]->points.size())/2));
	}
	return estimates;
}

tf::Transform CloudCorrespondenceMethods::getTransformFromPose(tf::Pose &p1,tf::Pose &p2)
{
  tf::Transform t;
  double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
  float linearposx1,linearposy1,linearposz1,linearposx2,linearposy2,linearposz2;
  boost::shared_ptr<tf::Quaternion> qtn;
  tf::Matrix3x3 mat;

  linearposx1 = p1.getOrigin().getX();
  linearposy1 = p1.getOrigin().getY();
  linearposz1 = p1.getOrigin().getZ();
  qtn.reset(new tf::Quaternion(p1.getRotation().getX(), p1.getRotation().getY(), p1.getRotation().getZ(), p1.getRotation().getW()));
  mat.setRotation(*qtn);
  mat.getRPY(roll1, pitch1, yaw1);

  linearposx2 = p2.getOrigin().getX();
  linearposy2 = p2.getOrigin().getY();
  linearposz2 = p2.getOrigin().getZ();
  qtn.reset(new tf::Quaternion(p2.getRotation().getX(), p2.getRotation().getY(), p2.getRotation().getZ(), p2.getRotation().getW()));
  mat.setRotation(*qtn);
  mat.getRPY(roll2, pitch2, yaw2);

  qtn->setRPY(roll1-roll2,pitch1-pitch2,yaw1-yaw2);
  tf::Vector3 v(linearposx1-linearposx2,linearposy1-linearposy2,linearposz1-linearposz2);
  t.setOrigin(v);
  t.setRotation(*qtn);
  //Eigen::Matrix4f m;
  //pcl_ros::transformAsMatrix(t,m);
  //cout<<m<<endl<<endl;
  return t;
}

////////////////////////////////////////////////////////////////////Helping Methods

visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, int id, std::string f_id, std::string ns="bounding_box", float r=0.5, float g=0.5, float b=0.5)
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

  marker.lifetime = ros::Duration(2);
  return marker;
}