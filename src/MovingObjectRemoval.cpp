#include "MOR/MovingObjectRemoval.h"

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

  marker.lifetime = ros::Duration(0.2);
  return marker;
}

////////////////////////////////////////////////////////////////////////

void MovingObjectDetectionCloud::groundPlaneRemoval(float x,float y,float z)
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

void MovingObjectDetectionCloud::groundPlaneRemoval()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr dsc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1,0.1,0.1);
    vg.filter(*dsc);

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr xyzi_tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    xyzi_tree->setInputCloud(cloud);

    std::vector<std::vector<int>> index_bank;
    for(int i=0;i<dsc->points.size();i++)
    {
      std::vector<int> ind;
      std::vector<float> dist;
      if(xyzi_tree->radiusSearch(dsc->points[i], 0.1, ind, dist) > 0 )
      //if(xyzi_tree->nearestKSearch(dsc->points[i], 20, ind, dist) > 0 )
      {
        if(ind.size()>3)
        {
          pcl::PointCloud<pcl::PointXYZI> temp;
          for(int j=0;j<ind.size();j++)
          {
            temp.points.push_back(cloud->points[ind[j]]);
          }
          temp.width = temp.points.size();
          temp.height = 1;

          Eigen::Vector4f cp;
          pcl::compute3DCentroid(temp, cp);
          Eigen::Matrix3f covariance_matrix;
          pcl::computeCovarianceMatrix(temp, cp, covariance_matrix);
          if(fabs(covariance_matrix(0,2))<0.001 && fabs(covariance_matrix(1,2))<0.001 && fabs(covariance_matrix(2,2))<0.001)
          {
            f_cloud->points.push_back(dsc->points[i]);
            index_bank.push_back(ind);
          }
        }
      }
    }

    std::unordered_map<float,std::vector<int>> bins;
    for(int i=0;i<f_cloud->points.size();i++)
    {
      float key = (float)((int)(f_cloud->points[i].z*10))/10;
      bins[key].push_back(i);
    }
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

    f_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_plane);
    extract.setNegative(true);
    extract.filter(*f_cloud);

    cloud=f_cloud;
}

void MovingObjectDetectionCloud::computeClusters(float distance_threshold, std::string f_id)
{
	clusters.clear();
	cluster_indices.clear();
	detection_results.clear();
  centroid_collection.reset(new pcl::PointCloud<pcl::PointXYZ>);
  #ifdef VISUALIZE
	cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
	#endif

  	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  	ec.setClusterTolerance(distance_threshold);
  	ec.setMinClusterSize(200);
  	ec.setMaxClusterSize(35000);
  	ec.setInputCloud(cloud);
  	ec.extract(cluster_indices);
	
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
	    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	    {
        #ifdef VISUALIZE
	    	cluster_collection->points.push_back(cloud->points[*pit]);
        #endif
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

  	for(int i=0;i<clusters.size();i++)
  	{
  		detection_results.push_back(false);
  	}
  	centroid_collection->width = centroid_collection->points.size();
	centroid_collection->height = 1;
	centroid_collection->is_dense = true;
  #ifdef VISUALIZE
  cluster_collection->width = cluster_collection->points.size();
	cluster_collection->height = 1;
	cluster_collection->is_dense = true;
  #endif
}

bool MovingObjectDetectionMethods::volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold)
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

void MovingObjectDetectionMethods::calculateCorrespondenceCentroid(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr fmp,double delta)
{
	pcl::CorrespondencesPtr ufmp(new pcl::Correspondences());

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputSource(fp);
  corr_est.setInputTarget(fc);
	corr_est.determineReciprocalCorrespondences(*ufmp);

	for(int j=0;j<ufmp->size();j++)
  	{
	    if(!volumeConstraint(c1[(*ufmp)[j].index_query],c2[(*ufmp)[j].index_match],0.3))
	    {
	      continue;
	    }

	    fmp->push_back((*ufmp)[j]);
  	}
}

std::vector<long> MovingObjectDetectionMethods::getClusterPointcloudChangeVector(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp,float resolution = 0.3f)
{
  std::vector<long> changed;
  srand((unsigned int)time(NULL));
  for(int j=0;j<mp->size();j++)
  {
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree_cd(resolution);
    octree_cd.setInputCloud(c1[(*mp)[j].index_query]);
    octree_cd.addPointsFromInputCloud();
    octree_cd.switchBuffers();
    octree_cd.setInputCloud(c2[(*mp)[j].index_match]);
    octree_cd.addPointsFromInputCloud();
    
    std::vector<int> newPointIdxVector;
    octree_cd.getPointIndicesFromNewVoxels(newPointIdxVector);
    changed.push_back(newPointIdxVector.size());
  }
  return changed;
}

std::vector<double> MovingObjectDetectionMethods::getPointDistanceEstimateVector(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp)
{
	std::vector<double> estimates;
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
			if((*corrs)[i].distance>0.005 && (*corrs)[i].distance<0.5)
			{
				count++;
			}
		}
		estimates.push_back(count/((c1[(*mp)[j].index_query]->points.size()+c2[(*mp)[j].index_match]->points.size())/2));
	}
	return estimates;
}

MovingObjectRemoval::MovingObjectRemoval(ros::NodeHandle nh_,std::string config_path,int n_bad,int n_good):nh(nh_),moving_confidence(n_bad),static_confidence(n_good)
{
    #ifdef VISUALIZE
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("bbox", 10);
    #endif

    #ifdef INTERNAL_SYNC
    raw_pc_sub.subscribe(nh, "/velodyne_points", 1);
    raw_odom_sub.subscribe(nh, "/camera/odom/sample", 1);
    sync_process.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),raw_pc_sub,raw_odom_sub));
    sync_process->registerCallback(boost::bind(&MovingObjectRemoval::movingCloudObjectSubscriber, this, _1, _2));
    #endif

    map_pc_sub.subscribe(nh, "/point_map", 1);
    map_odom_sub.subscribe(nh, "/icp_odom", 1);
    sync_update.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry>(map_pc_sub, map_odom_sub, 10));
    sync_update->registerCallback(boost::bind(&MovingObjectRemoval::mapUpdateSubscriber, this, _1, _2));

    ca.reset(new MovingObjectDetectionCloud());
    cb.reset(new MovingObjectDetectionCloud());
    mth.reset(new MovingObjectDetectionMethods());
}

void MovingObjectRemoval::movingCloudObjectSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
  clock_t begin_time = clock();
  std::cout<<"-----------------------------------------------------\n";
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(*input, cloud);
  pushRawCloudAndPose(cloud,odm->pose.pose);
  if(filterCloud(cloud,"/previous"))
  {
    #ifdef VISUALIZE
    pub.publish(output);
    #endif
  }

  std::cout<<1000.0*(clock()-begin_time)/CLOCKS_PER_SEC<<std::endl;
  std::cout<<"-----------------------------------------------------\n";
}

int MovingObjectRemoval::recurseFindClusterChain(int col,int track,std::vector<pcl::PointXYZ> &tv)
{
  tv.push_back(centroid_track[col]->points[track]);
  if(col == corrs_vec.size())
  {
    return track;
  }

  for(int j=0;j<corrs_vec[col]->size();j++)
  {
    if((*corrs_vec[col])[j].index_query == track)
    {
      if(res_vec[col+1][(*corrs_vec[col])[j].index_match] == true)
      {
        return recurseFindClusterChain(col+1,(*corrs_vec[col])[j].index_match,tv);
      }
      else
      {
        return -1;
      }
    }
  }
  return -1;
}

void MovingObjectRemoval::pushCentroid(std::vector<pcl::PointXYZ> &tv)
{
	for(int i=0;i<mo_vec.size();i++)
	{
		double dist = sqrt(pow(tv[moving_confidence-1].x-mo_vec[i].centroids[moving_confidence-1].x,2)+pow(tv[moving_confidence-1].y-mo_vec[i].centroids[moving_confidence-1].y,2)+pow(tv[moving_confidence-1].z-mo_vec[i].centroids[moving_confidence-1].z,2));
		if(dist<0.2)
		{
      mo_vec[i].centroids = tv;
			return;
		}
	}

	MovingObjectCentroid moc(tv,static_confidence);
	mo_vec.push_back(moc);
}

void MovingObjectRemoval::checkMovingClusterChain(pcl::CorrespondencesPtr mp,std::vector<bool> &res_ca,std::vector<bool> &res_cb,pcl::PointCloud<pcl::PointXYZ>::Ptr ca_cc, pcl::PointCloud<pcl::PointXYZ>::Ptr cb_cc)
{
  corrs_vec.push_back(mp);
  if(res_vec.size()!=0)
  {
    res_vec.pop_back();
  }
  res_vec.push_back(res_ca);
  res_vec.push_back(res_cb);
  
  if(centroid_track.size()==0)
  {
    centroid_track.push_back(ca_cc);
  }
  centroid_track.push_back(cb_cc);

  //std::cout<<corrs_vec.size()<<" "<<res_vec.size()<<" "<<centroid_track.size()<<std::endl;

  if(res_vec.size()%moving_confidence==0)
  {
    for(int i=0;i<res_vec[0].size();i++)
    {
      if(res_vec[0][i] == true)
      {
        std::vector<pcl::PointXYZ> tracking_vec;
      	int found_moving_index = recurseFindClusterChain(0,i,tracking_vec);
        if(found_moving_index != -1)
        {
          std::cout<<tracking_vec.size()<<std::endl;
          pushCentroid(tracking_vec);
        }
      }
    }
    corrs_vec.pop_front();
    res_vec.pop_front();
    centroid_track.pop_front();
  }
}

void MovingObjectRemoval::pushRawCloudAndPose(pcl::PCLPointCloud2 &in_cloud,geometry_msgs::Pose pose)
{
  ca = cb;
  cb.reset(new MovingObjectDetectionCloud());

  pcl::fromPCLPointCloud2(in_cloud, *(cb->cloud));

  tf::poseMsgToTF(pose,cb->ps);
  cb->groundPlaneRemoval(3.0,3.0,5.0);
  //cb->groundPlaneRemoval();
  cb->computeClusters(0.12,"single_cluster");
  cb->init = true;

  if(ca->init  == true && cb->init == true)
  {
    tf::Transform t = (cb->ps).inverseTimes(ca->ps);

    pcl::PointCloud<pcl::PointXYZ> temp = *ca->centroid_collection;
	pcl_ros::transformPointCloud(temp,*ca->centroid_collection,t);
	for(int i=0;i<ca->clusters.size();i++)
	{
		pcl::PointCloud<pcl::PointXYZI> temp;
		temp = *ca->clusters[i];
	  	pcl_ros::transformPointCloud(temp,*ca->clusters[i],t);
	}

  #ifdef VISUALIZE
	pcl::toPCLPointCloud2(*cb->cluster_collection,in_cloud);
	pcl_conversions::fromPCL(in_cloud, output);
	output.header.frame_id = "/current";
	pub.publish(output);
  #endif
  		
	pcl::CorrespondencesPtr mp(new pcl::Correspondences());
	  	
	//cluster correspondence methods (Global)
	mth->calculateCorrespondenceCentroid(ca->clusters,cb->clusters,ca->centroid_collection,cb->centroid_collection,mp,0.1);
	  	
	//moving object detection methods (Local)
	//std::vector<double> param_vec = mth->getPointDistanceEstimateVector(ca->clusters,cb->clusters,mp);
	std::vector<long> param_vec = mth->getClusterPointcloudChangeVector(ca->clusters,cb->clusters,mp,0.1);

	for(int j=0;j<mp->size();j++)
	{
		//std::cout<<"{"<<(*mp)[j].index_query<<"->"<<(*mp)[j].index_match<<"} Fit Score: "<<(*mp)[j].distance<<" Moving_Score: "<<param_vec[j]<<std::endl;
		long threshold = (ca->clusters[(*mp)[j].index_query]->points.size()+cb->clusters[(*mp)[j].index_match]->points.size())/25;
		//double threshold = 0.15;
		if(param_vec[j]>threshold)
		{
			ca->detection_results[(*mp)[j].index_query] = true;
			cb->detection_results[(*mp)[j].index_match] = true;
		}
		else
		{
			ca->detection_results[(*mp)[j].index_query] = false;
			cb->detection_results[(*mp)[j].index_match] = false;
		}
	}
	checkMovingClusterChain(mp,ca->detection_results,cb->detection_results,ca->centroid_collection,cb->centroid_collection);
  }
}

bool MovingObjectRemoval::filterCloud(pcl::PCLPointCloud2 &out_cloud,std::string f_id)
{
	xyz_tree.setInputCloud(cb->centroid_collection);
	float rd=0.8,gd=0.1,bd=0.4;int id = 1;
  pcl::PointIndicesPtr moving_points(new pcl::PointIndices);
	for(int i=0;i<mo_vec.size();i++)
	{
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		if(xyz_tree.nearestKSearch(mo_vec[i].centroids[moving_confidence-1], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
      #ifdef VISUALIZE
			marker_pub.publish(mark_cluster(cb->clusters[pointIdxNKNSearch[0]],id,"/current","bounding_box",rd,gd,bd));
			#endif

      for(int j=0;j<cb->cluster_indices[pointIdxNKNSearch[0]].indices.size();j++)
      {
        moving_points->indices.push_back(cb->cluster_indices[pointIdxNKNSearch[0]].indices[j]);
      }

			if(cb->detection_results[pointIdxNKNSearch[0]] == false || pointNKNSquaredDistance[0]>0.5)
			{
				if(mo_vec[i].decreaseConfidence())
				{
					mo_vec.erase(mo_vec.begin()+i);
					i--;
				}
			}
			else
			{
				mo_vec[i].increaseConfidence();
			}
			id++;
		}
	}

  pcl::PointCloud<pcl::PointXYZI>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cb->cloud);
  extract.setIndices(moving_points);
  extract.setNegative(true);
  extract.filter(*f_cloud);
  pcl::toPCLPointCloud2(*f_cloud,out_cloud);
  pcl_conversions::fromPCL(out_cloud, output);
  output.header.frame_id = f_id;
  return true;
}

void MovingObjectRemoval::mapUpdateSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{

}