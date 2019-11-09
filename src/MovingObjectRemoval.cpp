#include "MOR/MovingObjectRemoval.h"

//'xyz' -> refers to the variable with name xyz

////////////////////////////////////////////////////////////////////Helping Methods

visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, int id, std::string f_id, std::string ns="bounding_box", float r=0.5, float g=0.5, float b=0.5)
{
  /*Function to generate bounding box visualization markers. This function is used when the VISUALIZE
  flag is defined*/
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

  //colour of the box
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5; //opacity

  marker.lifetime = ros::Duration(0.2); //persistance duration
  return marker;
}

////////////////////////////////////////////////////////////////////////

void MovingObjectDetectionCloud::groundPlaneRemoval(float x,float y,float z)
{
    /*Hard coded ground plane removal*/

	  pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);
    pass.filter(*raw_cloud);
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);
    pass.filter(*raw_cloud);
    /*The pointcloud becomes more sparse as the distance of sampling from the lidar increases.
    So it has been trimmed in X,Y and Z directions*/

    pcl::CropBox<pcl::PointXYZI> cropBoxFilter (true);
    cropBoxFilter.setInputCloud(raw_cloud);
    Eigen::Vector4f min_pt(-x, -y, gp_limit, 1.0f);
    Eigen::Vector4f max_pt(x, y, z, 1.0f);
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);
    //cropBoxFilter.setNegative(true);
    cropBoxFilter.filter(*cloud); //'cloud' stores the pointcloud after removing ground plane
    gp_indices = cropBoxFilter.getRemovedIndices();
    /*ground plane is removed from 'raw_cloud' and their indices are stored in gp_indices*/
}

void MovingObjectDetectionCloud::groundPlaneRemoval(float x,float y)
{
    /*Voxel covariance based ground plane removal.*/

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);
    pass.filter(*raw_cloud);
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);
    pass.filter(*raw_cloud);
    /*The pointcloud becomes more sparse as the distance of sampling from the lidar increases.
    So it has been trimmed in X,Y and Z directions*/

    pcl::PointCloud<pcl::PointXYZI>::Ptr dsc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    /*pointcloud variables*/

    pcl::VoxelGrid<pcl::PointXYZI> vg; //voxelgrid filter to downsample input cloud
    vg.setInputCloud(raw_cloud);
    vg.setLeafSize(gp_leaf,gp_leaf,gp_leaf);
    vg.filter(*dsc); //'dsc' stores the downsampled pointcloud

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr xyzi_tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    xyzi_tree->setInputCloud(raw_cloud); //kdtree to search for NN of the down sampled cloud points

    std::vector<std::vector<int>> index_bank;
    /*stores the index of all the points in 'raw_cloud' which satisfies the covariance condition*/

    for(int i=0;i<dsc->points.size();i++)
    {
      std::vector<int> ind;
      std::vector<float> dist;
      if(xyzi_tree->radiusSearch(dsc->points[i], gp_leaf, ind, dist) > 0 )
      //if(xyzi_tree->nearestKSearch(dsc->points[i], 20, ind, dist) > 0 )
      {
        /*this can be radius search or nearest K search. most suitable one should be considered
        according to the results after experiments*/

        if(ind.size()>3) //atleast 3 points required for covariance matrix calculation
        {
          pcl::PointCloud<pcl::PointXYZI> temp;
          for(int j=0;j<ind.size();j++)
          {
            temp.points.push_back(raw_cloud->points[ind[j]]);
          }//put all NN (inside the voxel) into a temporary pointcloud
          temp.width = temp.points.size();
          temp.height = 1;

          Eigen::Vector4f cp;
          pcl::compute3DCentroid(temp, cp); //calculate centroid
          Eigen::Matrix3f covariance_matrix;
          pcl::computeCovarianceMatrix(temp, cp, covariance_matrix); //calculate 3D covariance matrix(3x3)
          if(fabs(covariance_matrix(0,2))<0.001 && fabs(covariance_matrix(1,2))<0.001 && fabs(covariance_matrix(2,2))<0.001)
          {
            /*
            xx|xy|xz
            yx|yy|yz
            zx|zy|zz
            covariance matrix: xz,yz and zz values should be less than a threshold.
            thresholds can be modified for better results depending on the type of pointcloud.
            */
            f_cloud->points.push_back(dsc->points[i]);
            index_bank.push_back(ind);
          }
        }
      }
    }

    std::unordered_map<float,std::vector<int>> bins;
    /*a bin holds all points having Z coordinate within a specific range*/
    
    for(int i=0;i<f_cloud->points.size();i++)
    {
      float key = (float)((int)(f_cloud->points[i].z*10))/bin_gap; //bin gap for the binning step
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
    /*search for the bin holding highest number of points. it is supposed to be the dominating 
    plane surface*/

    boost::shared_ptr<std::vector<int>> gp_i;
    pcl::PointIndicesPtr ground_plane(new pcl::PointIndices);
    for(int i=0;i<bins[tracked_key].size();i++)
    {
      for(int j=0;j<index_bank[bins[tracked_key][i]].size();j++)
      {
        gp_i->push_back(index_bank[bins[tracked_key][i]][j]); //store the ground plane point indices in 'gp_indices'
        ground_plane->indices.push_back(index_bank[bins[tracked_key][i]][j]);
      }
    }
    gp_indices = gp_i;

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(raw_cloud);
    extract.setIndices(ground_plane);
    extract.setNegative(true);
    extract.filter(*cloud);
    /*filter the pointcloud by removing ground plane*/
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
  /*initialize and clear the required variables*/

  	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  	ec.setClusterTolerance(distance_threshold);
  	ec.setMinClusterSize(min_cluster_size);
  	ec.setMaxClusterSize(max_cluster_size);
  	ec.setInputCloud(cloud);
  	ec.extract(cluster_indices);
	  /*euclidian clustering*/

  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>); //temporary variable
	    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	    {
        #ifdef VISUALIZE
	    	cluster_collection->points.push_back(cloud->points[*pit]);
        #endif
	    	cloud_cluster->points.push_back(cloud->points[*pit]); //extract the cluster into 'cloud_cluster'
	    }

	    cloud_cluster->header.frame_id = f_id;
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster); //add the cluster to a collection vector

	    Eigen::Vector4d temp;
	    pcl::compute3DCentroid(*cloud_cluster, temp); //compute centroid of the cluster
	    pcl::PointXYZ centroid;
	    centroid.x = temp[0]; centroid.y = temp[1]; centroid.z = temp[2];
	    centroid_collection->points.push_back(centroid); //add the centroid to a collection vector
  	}

  centroid_collection->width = centroid_collection->points.size();
  centroid_collection->height = 1;
  centroid_collection->is_dense = true;

  	for(int i=0;i<clusters.size();i++)
  	{
  		detection_results.push_back(false); 
      /*assign the moving detection results for all clusters as false initially*/
  	}

  #ifdef VISUALIZE 
  /*visualize the clustering results if VISUALIZE flag is defined*/
  cluster_collection->width = cluster_collection->points.size();
	cluster_collection->height = 1;
	cluster_collection->is_dense = true;
  #endif
}

bool MovingObjectDetectionMethods::volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold)
{
    /*check if two corresponding pointclouds have nearly equal volume*/

    Eigen::Vector4f min;
  	Eigen::Vector4f max;
  	double volp,volc;

  	pcl::getMinMax3D(*fp, min, max);
  	volp = (max[0]-min[0])*(max[1]-min[1])*(max[2]-min[2]);
  	pcl::getMinMax3D(*fc, min, max);
  	volc = (max[0]-min[0])*(max[1]-min[1])*(max[2]-min[2]);

  	if((abs(volp-volc)/(volp+volc))<threshold) 
  	{
      /*normalized volume difference should be less than threshold*/
  		return true;
  	}
  	return false;
}

void MovingObjectDetectionMethods::calculateCorrespondenceCentroid(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr fmp,double delta)
{
  /*finds the correspondence among the cluster centroids between two consecutive frames*/

	pcl::CorrespondencesPtr ufmp(new pcl::Correspondences());

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputSource(fp);
  corr_est.setInputTarget(fc);
	corr_est.determineReciprocalCorrespondences(*ufmp);
  /*euclidian distance based reciprocal correspondence (one to one correspondence)*/

	for(int j=0;j<ufmp->size();j++)
  	{
      /*filter the correspondences based on volume constraints and store in 'fmp'*/
	    if(!volumeConstraint(c1[(*ufmp)[j].index_query],c2[(*ufmp)[j].index_match],volume_constraint))
	    {
	      continue;
	    }

	    fmp->push_back((*ufmp)[j]);
  	}
}

std::vector<double> MovingObjectDetectionMethods::getClusterPointcloudChangeVector(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp,float resolution = 0.3f)
{
  /*builds the octree representation of the source and destination clouds. finds the
  number of new points appearing in the destination cloud with respect to the source
  cloud. repeats this for each pair of corresponding pointcloud clusters*/

  std::vector<double> changed;
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
    /*stores the indices of the new points appearing in the destination cluster*/

    octree_cd.getPointIndicesFromNewVoxels(newPointIdxVector);
    changed.push_back(newPointIdxVector.size());
  }
  return changed;
  /*return the movement scores*/
}

std::vector<double> MovingObjectDetectionMethods::getPointDistanceEstimateVector(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp)
{
  /*finds the correspondence of points from source to destination pointcloud.filters the
  correspondences having distance within a specific distance range. repeats this for each
  pair of corresponding pointcloud clusters*/

	std::vector<double> estimates;
 	pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> corr_est;
  pcl::CorrespondencesPtr corrs;    
	for(int j=0;j<mp->size();j++)
	{
		corrs.reset(new pcl::Correspondences());
    corr_est.setInputSource(c1[(*mp)[j].index_query]);
  	corr_est.setInputTarget(c2[(*mp)[j].index_match]);
		corr_est.determineCorrespondences(*corrs);
    /*euclidian distance based correspondence (one to many correspondence)*/

		double count = 0;
		for(int i=0;i<corrs->size();i++)
		{
			if((*corrs)[i].distance>pde_lb && (*corrs)[i].distance<pde_ub)
			{
				count++;
			}
		}
		estimates.push_back(count/((c1[(*mp)[j].index_query]->points.size()+c2[(*mp)[j].index_match]->points.size())/2));
		/*normalize 'count' with respect to the size of corresponding clusters*/
	}
	return estimates;
  /*return the movement scores*/
}

MovingObjectRemoval::MovingObjectRemoval(ros::NodeHandle nh_,std::string config_path,int n_bad,int n_good):nh(nh_),moving_confidence(n_bad),static_confidence(n_good)
{
    setVariables(config_path);

    /*ROS setup*/
    #ifdef VISUALIZE
    pub = nh.advertise<sensor_msgs::PointCloud2> (output_topic, 10);
    debug_pub = nh.advertise<sensor_msgs::PointCloud2> (debug_topic, 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>(marker_topic, 10);
    #endif

    #ifdef INTERNAL_SYNC
    pc_sub.subscribe(nh, input_pointcloud_topic, 1);
    odom_sub.subscribe(nh, input_odometry_topic, 1);
    sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),pc_sub,odom_sub));
    sync->registerCallback(boost::bind(&MovingObjectRemoval::movingCloudObjectSubscriber, this, _1, _2));
    /*internal message synchronization using ROS Approximate Time policy*/
    #endif

    ca.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); //previous pointcloud frame
    cb.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); //current pointcloud frame (latest)
    mth.reset(new MovingObjectDetectionMethods(volume_constraint,pde_lb,pde_ub));
    /*instantiate the shared pointers*/
}

void MovingObjectRemoval::movingCloudObjectSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
  /*subscriber for internal sync. works if INTERNAL_SYNC flag is defined*/

  clock_t begin_time = clock();
  std::cout<<"-----------------------------------------------------\n";
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(*input, cloud);
  pushRawCloudAndPose(cloud,odm->pose.pose);
  if(filterCloud(cloud,output_fid))
  {
    #ifdef VISUALIZE
    pub.publish(output);
    #endif
  }

  std::cout<<1000.0*(clock()-begin_time)/CLOCKS_PER_SEC<<std::endl;
  /*print CPU time taken by the algorithm per iteration*/

  std::cout<<"-----------------------------------------------------\n";
}

int MovingObjectRemoval::recurseFindClusterChain(int col,int track)
{
  /*takes the correspondence map buffer index and result buffer index as initial parameter
  and recurses till the end of all the correspondence maps available in the buffer. cluster chain
  information is obtained from the correspondence map buffer 'corrs_vec'. consistency in the 
  cluster chain is known using the result buffer 'res_vec'. returns -1 if consistency fails
  or else returns the index of the moving cluster in the cluster collection 'clusters' in
  the latest pointcloud frame 'cb'*/

  if(col == corrs_vec.size())
  {
    /*break condition for the recursion. return the index of the moving cluster*/
    return track;
  }

  for(int j=0;j<corrs_vec[col]->size();j++)
  {
    /*search all the unit correspondeces within the correspondence map in 'col' index of the buffer*/
    
    if((*corrs_vec[col])[j].index_query == track)
    {
      /*correspondence map should have the key 'track'*/

      if(res_vec[col+1][(*corrs_vec[col])[j].index_match] == true)
      {
        /*the mapping index must have a true positive value in the result buffer*/

        return recurseFindClusterChain(col+1,(*corrs_vec[col])[j].index_match);
        /*if both key and mapped index have true positive value then move for the next correspondence
        map in the buffer*/
      }
      else
      {
        return -1;
      }
    }
  }
  return -1;
}

void MovingObjectRemoval::pushCentroid(pcl::PointXYZ pt)
{
  /*function to check and push the moving centroid to the confirmed moving cluster vector*/

	for(int i=0;i<mo_vec.size();i++)
	{
    /*check if the moving centroid has already been added to the 'mo_vec' previously*/
		double dist = sqrt(pow(pt.x-mo_vec[i].centroid.x,2)+pow(pt.y-mo_vec[i].centroid.y,2)+pow(pt.z-mo_vec[i].centroid.z,2));
		if(dist<catch_up_distance)
		{
      /*if found a centroid close to the new moving centroid then return, as no additional 
      action is required*/
			return;
		}
	}

	MovingObjectCentroid moc(pt,static_confidence);
  /*assign static confidence to 'moc' that determines it's persistance in 'mo_vec'*/

	mo_vec.push_back(moc);
  /*if not present then add the new cluster centroid to the 'mo_vec'*/
}

void MovingObjectRemoval::checkMovingClusterChain(pcl::CorrespondencesPtr mp,std::vector<bool> &res_ca,std::vector<bool> &res_cb)
{
  /*gets new correspondence map and result vector after the detection step and updates the
  buffers. it checks for new moving clusters and adds them to the 'mo_vec'*/

  corrs_vec.push_back(mp);
  if(res_vec.size()!=0)
  {
    res_vec.pop_back(); //deletes the top most result in the buffer
  }
  res_vec.push_back(res_ca); //updates the buffer with the latest result
  res_vec.push_back(res_cb);
  
  if(res_vec.size()%moving_confidence==0)
  {
    for(int i=0;i<res_vec[0].size();i++)
    {
      if(res_vec[0][i] == true)
      {
        /*look to the historical data in the result buffer and check the clusters with true positive
        value in the first result vector within the buffer*/

      	int found_moving_index = recurseFindClusterChain(0,i);
        /*run the recursive test to find a potential cluster chain form the buffers*/

        if(found_moving_index != -1)
        {
          /*if found push the confirmed moving centroid into 'mo_vec'*/
          pushCentroid(cb->centroid_collection->points[found_moving_index]);
        }
      }
    }
    corrs_vec.pop_front(); //delete old values from the buffer
    res_vec.pop_front();
  }
}

void MovingObjectRemoval::pushRawCloudAndPose(pcl::PCLPointCloud2 &in_cloud,geometry_msgs::Pose pose)
{
  /*recieves the synchronized incoming data and runs detection methods*/

  ca = cb; //update previous frame with the current frame
  cb.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); //reset current frame

  pcl::fromPCLPointCloud2(in_cloud, *(cb->raw_cloud)); //load latest pointcloud
  tf::poseMsgToTF(pose,cb->ps); //load latest pose

  cb->groundPlaneRemoval(trim_x,trim_y,trim_z); //ground plane removal (hard coded)
  //cb->groundPlaneRemoval(trim_x,trim_y); //groud plane removal (voxel covariance)

  cb->computeClusters(ec_distance_threshold,"single_cluster"); 
  /*compute clusters within the lastet pointcloud*/

  cb->init = true; //confirm the frame for detection

  if(ca->init  == true && cb->init == true)
  {
    tf::Transform t = (cb->ps).inverseTimes(ca->ps); 
    /*calculate transformation matrix between previous and current pose. 't' transforms a point
    from the previous pose to the current pose*/

    pcl::PointCloud<pcl::PointXYZ> temp = *ca->centroid_collection;
	  pcl_ros::transformPointCloud(temp,*ca->centroid_collection,t);
    /*transform the previous centroid collection with respect to 't'*/

	for(int i=0;i<ca->clusters.size();i++)
	{
    /*transform the clusters in the collection vector of the previous frame with respect to 't'*/

		pcl::PointCloud<pcl::PointXYZI> temp;
		temp = *ca->clusters[i];
	  pcl_ros::transformPointCloud(temp,*ca->clusters[i],t);
	}

  #ifdef VISUALIZE //visualize the cluster collection if VISUALIZE flag is defined
	pcl::toPCLPointCloud2(*cb->cluster_collection,in_cloud);
	pcl_conversions::fromPCL(in_cloud, output);
	output.header.frame_id = debug_fid;
	debug_pub.publish(output);
  #endif
  		
	pcl::CorrespondencesPtr mp(new pcl::Correspondences()); 
  /*correspondence map between the cluster centroids of previous and current frame*/
	  	
	//cluster correspondence methods (Global)
	mth->calculateCorrespondenceCentroid(ca->clusters,cb->clusters,ca->centroid_collection,cb->centroid_collection,mp,0.1);
	/*calculate euclidian correspondence and apply the voulme constraint*/

	//moving object detection methods (Local)
  std::vector<double> param_vec;
  if(method_choice==1)
	{
    param_vec = mth->getPointDistanceEstimateVector(ca->clusters,cb->clusters,mp);
	}
  else if(method_choice==2)
  {
    param_vec = mth->getClusterPointcloudChangeVector(ca->clusters,cb->clusters,mp,0.1);
  }
  /*determine the movement scores for the corresponding clusters*/

	for(int j=0;j<mp->size();j++)
	{
		//std::cout<<"{"<<(*mp)[j].index_query<<"->"<<(*mp)[j].index_match<<"} Fit Score: "<<(*mp)[j].distance<<" Moving_Score: "<<param_vec[j]<<std::endl;
		double threshold;
    if(method_choice==1)
    {
      threshold = pde_distance_threshold;
		}
    else if(method_choice==2)
    {
      threshold = (ca->clusters[(*mp)[j].index_query]->points.size()+cb->clusters[(*mp)[j].index_match]->points.size())/opc_normalization_factor;
    }

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
    /*assign the boolean results acording to thresholds. true for moving and false for static cluster*/
	}
	checkMovingClusterChain(mp,ca->detection_results,cb->detection_results);
  /*submit the results to update the buffers*/
  }
}

bool MovingObjectRemoval::filterCloud(pcl::PCLPointCloud2 &out_cloud,std::string f_id)
{
  /*removes the moving objects from the latest pointcloud and puts the filtered cloud in 'output'.
  removes the static cluster centroids from 'mo_vec'*/

	xyz_tree.setInputCloud(cb->centroid_collection); 
  /*use kdtree for searching the moving cluster centroid within 'centroid_collection' of the
  latest frame*/

	float rd=0.8,gd=0.1,bd=0.4;int id = 1; //colour variables for visualizing red bounding box

  pcl::PointIndicesPtr moving_points(new pcl::PointIndices);
  /*stores the indices of the points belonging to the moving clusters within 'cloud'*/

	for(int i=0;i<mo_vec.size();i++)
	{
    /*iterate through all the moving cluster centroids*/

		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		if(xyz_tree.nearestKSearch(mo_vec[i].centroid, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
      /*search for the actual centroid in the centroid collection of the latest frame*/

      #ifdef VISUALIZE //visualize bounding box to show the moving cluster if VISUALIZE flag is defined
			marker_pub.publish(mark_cluster(cb->clusters[pointIdxNKNSearch[0]],id,debug_fid,"bounding_box",rd,gd,bd));
			#endif

      for(int j=0;j<cb->cluster_indices[pointIdxNKNSearch[0]].indices.size();j++)
      {
        /*add the indices of the moving clusters in 'cloud' to 'moving_points'*/
        moving_points->indices.push_back(cb->cluster_indices[pointIdxNKNSearch[0]].indices[j]);
      }

			if(cb->detection_results[pointIdxNKNSearch[0]] == false || pointNKNSquaredDistance[0]>leave_off_distance)
			{
        /*decrease the moving confidence if the cluster is found to be static in the latest results or
        if the cluster dosen't appear in the current frame*/

				if(mo_vec[i].decreaseConfidence())
				{
          /*remove the moving centroid from 'mo_vec' if the confidence reduces to 0*/
					mo_vec.erase(mo_vec.begin()+i);
					i--;
				}
			}
			else
			{
				mo_vec[i].centroid = cb->centroid_collection->points[pointIdxNKNSearch[0]];
        /*update the moving centroid with the latest centroid of the moving cluster*/

				mo_vec[i].increaseConfidence(); //increase the moving confidence
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
  /*extract the moving clusters from 'cloud' and assign the filtered cloud to 'f_cloud'*/

  for(int i=0;i<cb->gp_indices->size();i++)
  {
    f_cloud->points.push_back(cb->raw_cloud->points[cb->gp_indices->at(i)]);
  }
  f_cloud->width = f_cloud->points.size();
  f_cloud->height = 1;
  f_cloud->is_dense = true;
  /*merge the ground plane to the filtered cloud*/

  pcl::toPCLPointCloud2(*f_cloud,out_cloud);
  pcl_conversions::fromPCL(out_cloud, output);
  output.header.frame_id = f_id;
  /*assign the final filtered cloud to the 'output'*/

  return true; //confirm that a new filtered cloud is available
}

void MovingObjectRemoval::setVariables(std::string config_file_path)
{
  std::fstream config;
  config.open(config_file_path);

  if(!config.is_open())
  {
    std::cout<<"Couldnt open the file\n";
    exit(0);
  } //open config file

  std::string line,parm1,parm2; //required string variables
  while(std::getline(config,line)) //extract lines one by one
  {
    if(line[0]=='#' || line.length()<3)
    {
      continue;
    }
    parm1 = "";parm2="";
    bool flag = true;
    for(int ind=0;ind<line.length();ind++)
    {
      if(line[ind]==':')
      {
        flag = false;
        continue;
      }
      if(flag)
      {
        parm1.push_back(line[ind]);
      }
      else
      {
        parm2.push_back(line[ind]);
      }
    } //extract lines "name_of_variable:value"

      std::cout<<parm1<<":";
      if(parm1 == "gp_limit")
      {
        gp_limit = std::stof(parm2);
        std::cout<<gp_limit;
      }
      else if(parm1 == "gp_leaf")
      {
        gp_leaf = std::stof(parm2);
        std::cout<<gp_leaf;
      }
      else if(parm1 == "bin_gap")
      {
        bin_gap = std::stof(parm2);
        std::cout<<bin_gap;
      }
      else if(parm1 == "min_cluster_size")
      {
        min_cluster_size = std::stol(parm2);
        std::cout<<min_cluster_size;
      }
      else if(parm1 == "max_cluster_size")
      {
        max_cluster_size = std::stol(parm2);
        std::cout<<max_cluster_size;
      }
      else if(parm1 == "volume_constraint")
      {
        volume_constraint = std::stof(parm2);
        std::cout<<volume_constraint;
      }
      else if(parm1 == "pde_lb")
      {
        pde_lb = std::stof(parm2);
        std::cout<<pde_lb;
      }
      else if(parm1 == "pde_ub")
      {
        pde_ub = std::stof(parm2);
        std::cout<<pde_ub;
      }
      else if(parm1 == "output_topic")
      {
        output_topic = parm2;
        std::cout<<output_topic;
      }
      else if(parm1 == "debug_topic")
      {
        debug_topic = parm2;
        std::cout<<debug_topic;
      }
      else if(parm1 == "marker_topic")
      {
        marker_topic = parm2;
        std::cout<<marker_topic;
      }
      else if(parm1 == "input_pointcloud_topic")
      {
        input_pointcloud_topic = parm2;
        std::cout<<input_pointcloud_topic;
      }
      else if(parm1 == "input_odometry_topic")
      {
        input_odometry_topic = parm2;
        std::cout<<input_odometry_topic;
      }
      else if(parm1 == "output_fid")
      {
        output_fid = parm2;
        std::cout<<output_fid;
      }
      else if(parm1 == "debug_fid")
      {
        debug_fid = parm2;
        std::cout<<debug_fid;
      }
      else if(parm1 == "leave_off_distance")
      {
        leave_off_distance = std::stof(parm2);
        std::cout<<leave_off_distance;
      }
      else if(parm1 == "catch_up_distance")
      {
        catch_up_distance = std::stof(parm2);
        std::cout<<catch_up_distance;
      }
      else if(parm1 == "trim_x")
      {
        trim_x = std::stof(parm2);
        std::cout<<trim_x;
      }
      else if(parm1 == "trim_y")
      {
        trim_y = std::stof(parm2);
        std::cout<<trim_y;
      }
      else if(parm1 == "trim_z")
      {
        trim_z = std::stof(parm2);
        std::cout<<trim_z;
      }
      else if(parm1 == "ec_distance_threshold")
      {
        ec_distance_threshold = std::stof(parm2);
        std::cout<<ec_distance_threshold;
      }
      else if(parm1 == "opc_normalization_factor")
      {
        opc_normalization_factor = std::stof(parm2);
        std::cout<<opc_normalization_factor;
      }
      else if(parm1 == "pde_distance_threshold")
      {
        pde_distance_threshold = std::stof(parm2);
        std::cout<<pde_distance_threshold;
      }
      else if(parm1 == "method_choice")
      {
        method_choice = std::stoi(parm2);
        std::cout<<method_choice;
      }
      else
      {
        std::cout<<"Invalid parameter found in config file\n";
        exit(0);
      }
      std::cout<<std::endl;
      /*assign values to the variables based on name*/
  }
}