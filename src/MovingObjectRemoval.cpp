#include "MOR/MovingObjectRemoval.h"

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr MovingObjectDetectionCloud::tree = boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZI>>(new pcl::KdTreeFLANN<pcl::PointXYZI>);
extern ros::Publisher pub,marker_pub;

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

void MovingObjectDetectionCloud::computeClusters(float distance_threshold, string f_id)
{
	clusters.clear();
	cluster_indices.clear();
	detection_results.clear();
  centroid_collection.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
	
  	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  	ec.setClusterTolerance(distance_threshold);
  	ec.setMinClusterSize(200);
  	ec.setMaxClusterSize(35000);
  	ec.setInputCloud(cloud);
  	ec.extract(cluster_indices);
	
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

  	for(int i=0;i<clusters.size();i++)
  	{
  		detection_results.push_back(false);
  	}
  centroid_collection->width = centroid_collection->points.size();
	centroid_collection->height = 1;
	centroid_collection->is_dense = true;
  cluster_collection->width = cluster_collection->points.size();
	cluster_collection->height = 1;
	cluster_collection->is_dense = true;
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

void MovingObjectDetectionMethods::calculateCorrespondenceCentroid(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr fmp,double delta)
{
	pcl::CorrespondencesPtr ufmp(new pcl::Correspondences());

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputSource(fp);
  corr_est.setInputTarget(fc);
	corr_est.determineReciprocalCorrespondences(*ufmp);

	for(int j=0;j<ufmp->size();j++)
  	{
	    if(!volumeConstraint(c1[(*ufmp)[j].index_query],c2[(*ufmp)[j].index_match],0.5))
	    {
	      cout<<"Cluster Skipped!\n";
	      continue;
	    }

	    fmp->push_back((*ufmp)[j]);
  	}
}

vector<long> MovingObjectDetectionMethods::getClusterPointcloudChangeVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp,float resolution = 0.3f)
{
  cout<<"\n ********* PC Change *********** \n";
  vector<long> changed;
  srand((unsigned int)time(NULL));
  for(int j=0;j<mp->size();j++)
  {
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

vector<double> MovingObjectDetectionMethods::getPointDistanceEstimateVector(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, pcl::CorrespondencesPtr mp)
{
  cout<<"\n ********* Distance Estimate *********** \n";
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
			if((*corrs)[i].distance>0.005 && (*corrs)[i].distance<0.5)
			{
				count++;
			}
		}
		estimates.push_back(count/((c1[(*mp)[j].index_query]->points.size()+c2[(*mp)[j].index_match]->points.size())/2));
	}
	return estimates;
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

////////////////////////////////////////////////////////////////////////

MovingObjectRemoval::MovingObjectRemoval(int n_bad,int n_good)
{
    moving_confidence = n_bad;
    static_confidence = n_good;

    ca.reset(new MovingObjectDetectionCloud());
    cb.reset(new MovingObjectDetectionCloud());
    mth.reset(new MovingObjectDetectionMethods());
}

bool MovingObjectRemoval::recurseFindClusterChain(int col,int track)
{
  if(col == corrs_vec.size())
  {
    return true;
  }

  for(int j=0;j<corrs_vec[col]->size();j++)
  {
    if((*corrs_vec[col])[j].index_query == track)
    {
      if(res_vec[col+1][(*corrs_vec[col])[j].index_match] == true)
      {
        return recurseFindClusterChain(col+1,(*corrs_vec[col])[j].index_match);
      }
      else
      {
        return false;
      }
    }
  }
  return false;
}

void MovingObjectRemoval::checkMovingClusterChain(pcl::CorrespondencesPtr mp,vector<bool> &res_ca,vector<bool> &res_cb)
{
  corrs_vec.push_back(mp);
  if(res_vec.size()==0)
  {
    res_vec.push_back(res_ca);
  }
  res_vec.push_back(res_cb);
    
  if(res_vec.size()%moving_confidence==0)
  {
    for(int i=0;i<res_vec[0].size();i++)
    {
      if(res_vec[0][i] == true)
      {
        if(recurseFindClusterChain(0,i) == true)
        {
          //add moving centroid to mo_vec
        }
      }
    }
    corrs_vec.erase(corrs_vec.begin());
    res_vec.erase(res_vec.begin());
  }
}

void MovingObjectRemoval::pushRawCloudAndPose(pcl::PCLPointCloud2 &cloud,geometry_msgs::Pose pose)
{
  ca = cb;
  cb.reset(new MovingObjectDetectionCloud());

  pcl::fromPCLPointCloud2(cloud, *(cb->cloud));

  tf::poseMsgToTF(pose,cb->ps);
  cb->groundPlaneRemoval(4.0,4.0,5.0);
  cb->init = true;

  if(ca->init  == true && cb->init == true)
  {
      clock_t begin_time = clock();
      tf::Transform t = (cb->ps).inverseTimes(ca->ps);

      pcl::PointCloud<pcl::PointXYZI> temp = *ca->cloud;
      pcl_ros::transformPointCloud(temp,*ca->cloud,t);

      ca->computeClusters(0.11,"single_cluster");
      cb->computeClusters(0.11,"single_cluster");
      
      pcl::toPCLPointCloud2(*ca->cluster_collection,cloud);
      pcl_conversions::fromPCL(cloud, output);
      output.header.frame_id = "previous";
      pub.publish(output);
      pcl::toPCLPointCloud2(*cb->cluster_collection,cloud);
      pcl_conversions::fromPCL(cloud, output);
      output.header.frame_id = "current";
      pub.publish(output);
        
      pcl::CorrespondencesPtr mp(new pcl::Correspondences());
        
      //cluster correspondence methods (Global)
      mth->calculateCorrespondenceCentroid(ca->clusters,cb->clusters,ca->centroid_collection,cb->centroid_collection,mp,0.1);
        
      //moving object detection methods (Local)
      //vector<double> param_vec = mth->getPointDistanceEstimateVector(ca->clusters,cb->clusters,mp);
      vector<long> param_vec = mth->getClusterPointcloudChangeVector(ca->clusters,cb->clusters,mp,0.1);

      float rs=0.4,gs=0.6,bs=0.8,rd=0.8,gd=0.1,bd=0.4;int id = 1;
      for(int j=0;j<mp->size();j++)
      {
        cout<<"{"<<(*mp)[j].index_query<<"->"<<(*mp)[j].index_match<<"} Fit Score: "<<(*mp)[j].distance<<" Moving_Score: "<<param_vec[id-1]<<endl;
        long threshold = (ca->clusters[(*mp)[j].index_query]->points.size()+cb->clusters[(*mp)[j].index_match]->points.size())/25;
        //double threshold = 0.15;
        if(param_vec[id-1]>threshold)
        {
          marker_pub.publish(mark_cluster(ca->clusters[(*mp)[j].index_query],id,"previous","bounding_box",rd,gd,bd));
          marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id,"current","bounding_box",rd,gd,bd));
          ca->detection_results[(*mp)[j].index_query] = true;
          cb->detection_results[(*mp)[j].index_match] = true;
        }
        else
        {
          marker_pub.publish(mark_cluster(ca->clusters[(*mp)[j].index_query],id,"previous","bounding_box",rs,gs,bs));
          marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id,"current","bounding_box",rs,gs,bs));
        }
        id++;
      }
      cout<<"-----------------------------------------------------\n";
      cout<<1000.0*(clock()-begin_time)/CLOCKS_PER_SEC<<endl;

      checkMovingClusterChain(mp,ca->detection_results,cb->detection_results);
  }
}

void MovingObjectRemoval::getFilteredCloud(sensor_msgs::PointCloud2 &out_cloud)
{

}