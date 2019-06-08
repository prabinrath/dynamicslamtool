#include "CC/CloudCorrespondence.h"

pcl::search::KdTree<pcl::PointXYZI>::Ptr CloudCorrespondence::tree = boost::shared_ptr<pcl::search::KdTree<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);

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

void CloudCorrespondence::removePlaneSurface(float distance_threshold)
{
	pcl::SACSegmentation<pcl::PointXYZI> seg;
  	seg.setOptimizeCoefficients(true);
  	seg.setModelType(pcl::SACMODEL_PLANE);
  	seg.setMethodType(pcl::SAC_RANSAC);
  	seg.setMaxIterations(100);
  	seg.setDistanceThreshold(distance_threshold);

  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>());
  	int nr_points = (int) cloud->points.size();
  	while (cloud->points.size () > 0.3 * nr_points)
  	{
	    // Segment the largest planar component from the remaining cloud
	    seg.setInputCloud(cloud);
	    seg.segment(*inliers, *coefficients);
	    if (inliers->indices.size () == 0)
	    {
	      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	      break;
	    }
	    pcl::ExtractIndices<pcl::PointXYZI> extract;
	    extract.setInputCloud(cloud);
	    extract.setIndices(inliers);
	    // Remove the planar inliers, extract the rest
	    extract.setNegative (true);
	    extract.filter (*cloud_f);
	    *cloud = *cloud_f;
	}
}

void CloudCorrespondence::computeClusters(float distance_threshold, string f_id)
{
	feature_bank.clear();
	clusters.clear();
	cluster_indices.clear();
	cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
	
  	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  	ec.setClusterTolerance(distance_threshold);
  	ec.setMinClusterSize(150);
  	ec.setMaxClusterSize(25000);
  	tree->setInputCloud(cloud);
  	ec.setSearchMethod(tree);
  	ec.setInputCloud(cloud);
  	ec.extract(cluster_indices);

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

	    clusters.push_back(cloud_cluster);
	    /*///////////////////////////////////////////// Feature Extraction : Viewpoint Feature Histogram
	  	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	  	pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> norm_est;
  		//norm_est.setKSearch(10);	//K nearest neighbours, radius search can also be used
  		norm_est.setRadiusSearch(0.1);
	  	norm_est.setInputCloud(cloud_cluster);
	  	norm_est.compute(*normals);

	  	pcl::VFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::VFHSignature308> vfh;
	  	vfh.setInputCloud(cloud_cluster);
	  	vfh.setInputNormals(normals);
	  	pcl::search::KdTree<pcl::PointXYZI>::Ptr _tree(new pcl::search::KdTree<pcl::PointXYZI>());
	  	vfh.setSearchMethod(_tree);
	  	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
	  	vfh.compute(*vfhs);
	  	//std::cout<<vfhs->points[0]<<std::endl;

	  	//pcl::visualization::PCLHistogramVisualizer viewer;
	  	//viewer.addFeatureHistogram(*vfhs, 308); 
	  	//viewer.spinOnce(1000);
		vector<double> vec;
	  	for(int i=0;i<308;i++)
	  	{
	  		vec.push_back(vfhs->points[0].histogram[i]);
	  	}
	  	/*//////////////////////////////////////////////////////////////////////

	    Eigen::Vector4f centroid;
	    pcl::compute3DCentroid(*cloud_cluster, centroid);
	    vector<double> vec;
	    vec.push_back(centroid[0]);vec.push_back(centroid[1]);vec.push_back(centroid[2]);

		feature_bank.push_back(vec);
  	}

  	cluster_collection->width = cluster_collection->points.size();
	cluster_collection->height = 1;
	cluster_collection->is_dense = true;
}

void CloudCorrespondenceMethods::swap_check_correspondence_centroidKdtree(vector<vector<double>> &fp,vector<vector<double>> &fc, map<int,pair<int,double>> &mp, double delta)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cldp(new pcl::PointCloud<pcl::PointXYZ>());

	for(int i=0;i<fp.size();i++)
	{
		pcl::PointXYZ pt;
		pt.x = fp[i][0];
		pt.y = fp[i][1];
		pt.z = fp[i][2];
		cldp->points.push_back(pt);
	} 

	pcl::KdTreeFLANN<pcl::PointXYZ,flann::ChiSquareDistance<float>>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(cldp);

	for(int i=0;i<fc.size();i++)
	{
		pcl::PointXYZ pt;
		pt.x = fc[i][0];
		pt.y = fc[i][1];
		pt.z = fc[i][2];

		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);

		if(kdtree->nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  		{
  			if(pointNKNSquaredDistance[0]<delta)
  			{
	  			if(mp.find(pointIdxNKNSearch[0]) == mp.end())
	  			{
	  				mp[pointIdxNKNSearch[0]] = {i,pointNKNSquaredDistance[0]};
	  			}
	  			else
	  			{
	  				if(mp[pointIdxNKNSearch[0]].second>pointNKNSquaredDistance[0])
	  				{
	  					mp[pointIdxNKNSearch[0]] = {i,pointNKNSquaredDistance[0]};
	  				}
	  			}
  			}
  		}		
	}
}

void CloudCorrespondenceMethods::calculate_correspondence_centroidKdtree(vector<vector<double>> &fp, vector<vector<double>> &fc, map<int,pair<int,double>> &mp,double delta)
{
	cout<<fp.size()<<" Centroid "<<fc.size()<<endl;
	map<int,pair<int,double>> m1,m2;

	swap_check_correspondence_centroidKdtree(fp,fc,m1,delta);
	swap_check_correspondence_centroidKdtree(fc,fp,m2,delta);

	for(map<int,pair<int,double>>::iterator it1 = m1.begin();it1!=m1.end();it1++)
	{
		map<int,pair<int,double>>::iterator fnd = m2.find(it1->second.first);
		if(fnd != m2.end())
		{
			if(fnd->second.first == it1->first)
			{
				mp[it1->first] = {it1->second.first,min(it1->second.second,fnd->second.second)};
			}
		}
	}
}

void CloudCorrespondenceMethods::calculate_correspondence_dtw(vector<vector<double>> &fp, vector<vector<double>> &fc, map<int,pair<int,double>> &mp, double delta)
{
	cout<<fp.size()<<" DTW "<<fc.size()<<endl;
	for(int i=0;i<fc.size();i++)
	{
		LB_Improved filter(fc[i], fc[i].size() / 10);
		double bestfit = filter.getLowestCost();
		int ind = 0;
		for(int j=0;j<fp.size();j++)
		{
			double curr = filter.test(fp[j]);
			if(curr<bestfit)
			{
				bestfit = curr;
				ind = j;
			}
		}
		if(bestfit<delta)
		{
			if(mp.find(ind)==mp.end())
			{
				mp[ind]={i,bestfit};
			}
			else
			{
				if(mp[ind].second>bestfit)
				{
					mp[ind]={i,bestfit};
				}
			}
		}
	}	
}

void CloudCorrespondenceMethods::swap_check_correspondence_VFHkdtree(vector<vector<double>> &fp,vector<vector<double>> &fc, map<int,pair<int,double>> &mp, double delta)
{
	pcl::PointCloud<pcl::VFHSignature308>::Ptr cldp(new pcl::PointCloud<pcl::VFHSignature308>());
	for(int i=0;i<fp.size();i++)
	{
		pcl::VFHSignature308 pt;
		for(int j=0;j<fp[i].size();j++)
		{
			pt.histogram[j] = fp[i][j];
		}
		cldp->points.push_back(pt);
	}
	
	pcl::KdTreeFLANN<pcl::VFHSignature308,flann::ChiSquareDistance<float>>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::VFHSignature308>);
	kdtree->setInputCloud(cldp);

	for(int i=0;i<fc.size();i++)
	{
		pcl::VFHSignature308 pt;
		for(int j=0;j<fc[i].size();j++)
		{
			pt.histogram[j] = fc[i][j];
		}

		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);

		if(kdtree->nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  		{
  			if(pointNKNSquaredDistance[0]<delta)
  			{
	  			if(mp.find(pointIdxNKNSearch[0]) == mp.end())
	  			{
	  				mp[pointIdxNKNSearch[0]] = {i,pointNKNSquaredDistance[0]};
	  			}
	  			else
	  			{
	  				if(mp[pointIdxNKNSearch[0]].second>pointNKNSquaredDistance[0])
	  				{
	  					mp[pointIdxNKNSearch[0]] = {i,pointNKNSquaredDistance[0]};
	  				}
	  			}
  			}
  		}
	}
}

void CloudCorrespondenceMethods::calculate_correspondence_VFHkdtree(vector<vector<double>> &fp,vector<vector<double>> &fc, map<int,pair<int,double>> &mp, double delta)
{
	cout<<fp.size()<<" KdTree "<<fc.size()<<endl;
	map<int,pair<int,double>> m1,m2;

	swap_check_correspondence_VFHkdtree(fp,fc,m1,delta);
	swap_check_correspondence_VFHkdtree(fc,fp,m2,delta);

	for(map<int,pair<int,double>>::iterator it1 = m1.begin();it1!=m1.end();it1++)
	{
		map<int,pair<int,double>>::iterator fnd = m2.find(it1->second.first);
		if(fnd != m2.end())
		{
			if(fnd->second.first == it1->first)
			{
				mp[it1->first] = {it1->second.first,min(it1->second.second,fnd->second.second)};
			}
		}
	}
}

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

  marker.lifetime = ros::Duration(3);
  return marker;
}