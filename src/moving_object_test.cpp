#include "CC/CloudCorrespondence.h"
#include <ctime>
#include <cmath>

extern visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, int id, string f_id, string ns="bounding_box", float r=0.5, float g=0.5, float b=0.5);
extern visualization_msgs::Marker mark_direction(double x,double y,double z,int id, string f_id, string ns="direction_vector", float r=0.5, float g=0.5, float b=0.5);
extern ros::Publisher pub,marker_pub;

boost::shared_ptr<CloudCorrespondence> ca,cb;
boost::shared_ptr<CloudCorrespondenceMethods> mth;

void moving_object_test(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
	sensor_msgs::PointCloud2 output;
  	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  	pcl_conversions::toPCL(*input, *cloud);
  
  	ca = cb;
  	cb.reset(new CloudCorrespondence());
  	pcl::fromPCLPointCloud2(*cloud, *(cb->cloud));

  	tf::poseMsgToTF(odm->pose.pose,cb->ps);
  	cb->filterPassThrough(4.0,4.0,5.0);
	cb->init = true;

	if(ca->init  == true && cb->init == true)
	{
		clock_t begin_time = clock();
		tf::Transform t = (cb->ps).inverseTimes(ca->ps);

    	pcl::PointCloud<pcl::PointXYZI> temp = *ca->cloud;
	  	pcl_ros::transformPointCloud(temp,*ca->cloud,t);

	  	ca->computeClusters(0.11,"single_cluster");
		cb->computeClusters(0.11,"single_cluster");
		
		pcl::toPCLPointCloud2(*ca->cluster_collection,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "previous";
		pub.publish(output);
		pcl::toPCLPointCloud2(*cb->cluster_collection,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "current";
		pub.publish(output);
  		
	  	pcl::CorrespondencesPtr mp(new pcl::Correspondences());
	  	
	  	//cluster correspondence methods (Global)
	  	mth->calculateCorrespondenceCentroid(ca->centroid_collection,cb->centroid_collection,mp,0.1);
	  	
	  	//moving object detection methods (Local)
	  	vector<double> param_vec = mth->getPointDistanceEstimateVector(ca->clusters,cb->clusters,mp);
	  	//vector<long> param_vec = mth->getClusterPointcloudChangeVector(ca->clusters,cb->clusters,mp,0.1);

	  	float rs=0.4,gs=0.6,bs=0.8,rd=0.8,gd=0.1,bd=0.4;int id = 1;
	  	for(int j=0;j<mp->size();j++)
		{
			cout<<"{"<<(*mp)[j].index_query<<"->"<<(*mp)[j].index_match<<"} Fit Score: "<<(*mp)[j].distance<<" Moving_Score: "<<param_vec[id-1]<<endl;
			//long threshold = (ca->clusters[(*mp)[j].index_query]->points.size()+cb->clusters[(*mp)[j].index_match]->points.size())/25;
			double threshold = 0.1;
			if(param_vec[id-1]>threshold)
			{
				marker_pub.publish(mark_cluster(ca->clusters[(*mp)[j].index_query],id,"previous","bounding_box",rd,gd,bd));
				marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id,"current","bounding_box",rd,gd,bd));
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
	}
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test_moving_object");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 10);
  marker_pub = nh.advertise<visualization_msgs::Marker>("bbox", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/camera/odom/sample", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
  sync.registerCallback(boost::bind(&moving_object_test, _1, _2));

  ca.reset(new CloudCorrespondence());
  cb.reset(new CloudCorrespondence());
  mth.reset(new CloudCorrespondenceMethods());

  ros::spin();
}
		//Full cloud approach
		/*pcl::PointCloud<pcl::PointXYZI> temp = *ca->cloud;
	  	pcl_ros::transformPointCloud(temp,*ca->cloud,t);

		pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI> sift;
    	sift.setRadiusSearch(0.1);
    	sift.setScales(0.01, 3, 4);
    	sift.setMinimumContrast(0.001);

    
    	sift.setInputCloud(ca->cloud);
		sift.compute(*ca->cluster_collection);
		sift.setInputCloud(cb->cloud);
		sift.compute(*cb->cluster_collection);

		pcl::CorrespondencesPtr corrs(new pcl::Correspondences ());
  		pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> corr_est;
  		corr_est.setInputSource(ca->cluster_collection);
  		corr_est.setInputTarget(cb->cluster_collection);
		corr_est.determineCorrespondences(*corrs);

		pcl::PointCloud<pcl::PointXYZI> corrs_ca,corrs_cb;
		for(int i=0;i<corrs->size();i++)
		{
			if((*corrs)[i].distance>0.01 && (*corrs)[i].distance<0.5 && mth->densityConstraint(ca->cluster_collection->points[(*corrs)[i].index_query],ca->cluster_collection,ca->tree,5))
			{
				corrs_ca.points.push_back(ca->cluster_collection->points[(*corrs)[i].index_query]);
				corrs_cb.points.push_back(cb->cluster_collection->points[(*corrs)[i].index_match]);
				cout<<" Distance: "<<(*corrs)[i].distance<<endl;
			}
		}
		corrs_ca.width = corrs_ca.points.size();
		corrs_ca.height = 1;
		corrs_ca.is_dense = true;
		corrs_cb.width = corrs_cb.points.size();
		corrs_cb.height = 1;
		corrs_cb.is_dense = true;

		pcl::toPCLPointCloud2(corrs_ca,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "previous";
		pub.publish(output);
		pcl::toPCLPointCloud2(corrs_cb,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "current";
		pub.publish(output);
		*/
		/*
		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_t(new pcl::PointCloud<pcl::PointXYZI>),rotated_p(new pcl::PointCloud<pcl::PointXYZI>);

		pcl_ros::transformPointCloud(*ca->cloud,*keypoints_t,t);

		pcl::CorrespondencesPtr corrs(new pcl::Correspondences ());
  		pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> corr_est;
  		corr_est.setInputSource(keypoints_t);
  		corr_est.setInputTarget(cb->cloud);
		corr_est.determineCorrespondences(*corrs);

		pcl::PointCloud<pcl::PointXYZI> corrs_ca;
		pcl::PointCloud<pcl::PointXYZ> dir_cloud;

		tf::Transform rot;
		rot.setIdentity();
		rot.setRotation(t.getRotation());
		pcl_ros::transformPointCloud(*ca->cloud,*rotated_p,rot);

		for(int i=0;i<corrs->size();i++)
		{
			if((*corrs)[i].distance<0.01)
			{
				corrs_ca.points.push_back(rotated_p->points[(*corrs)[i].index_query]);
				pcl::PointXYZ dir = mth->getDirectionVector(rotated_p->points[(*corrs)[i].index_query],cb->cloud->points[(*corrs)[i].index_match]); 
				dir_cloud.points.push_back(dir);
				//cout<<"Direction: "<<dir.x<<" "<<dir.y<<" "<<dir.z<<" Distance: "<<(*corrs)[i].distance<<endl;
			}
		}
		dir_cloud.width = dir_cloud.points.size();
		dir_cloud.height = 1;
		dir_cloud.is_dense = true;
		corrs_ca.width = corrs_ca.points.size();
		corrs_ca.height = 1;
		corrs_ca.is_dense = true;

		pcl::PointCloud<pcl::PointXYZ> dir_cloud_f;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(dir_cloud.makeShared());
		sor.setMeanK(10);
		sor.setStddevMulThresh(0.1);
		sor.filter(dir_cloud_f);
		Eigen::Vector4f cp;
		pcl::compute3DCentroid(dir_cloud_f, cp);
		Eigen::Matrix3f covariance_matrix;
		pcl::computeCovarianceMatrix (dir_cloud_f, cp, covariance_matrix); 
		cout<<covariance_matrix<<endl;
		marker_pub.publish(mark_direction(cp(1),cp(2),cp(0),1,"direction","direction_vector",0,1,0));

		pcl::toPCLPointCloud2(corrs_ca,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "previous";
		pub.publish(output);
		pcl::toPCLPointCloud2(dir_cloud_f,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "direction";
		pub.publish(output);
		*/

		//Cluster first approach
		/*pcl::PointCloud<pcl::PointXYZI> temp = *ca->cloud;
	  	pcl_ros::transformPointCloud(temp,*ca->cloud,t);

	  	ca->computeClusters(0.11,"single_cluster");
		cb->computeClusters(0.11,"single_cluster");
		
		pcl::toPCLPointCloud2(*ca->cluster_collection,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "previous";
		pub.publish(output);
		pcl::toPCLPointCloud2(*cb->cluster_collection,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "current";
		pub.publish(output);
  		
	  	pcl::CorrespondencesPtr mp(new pcl::Correspondences());
	  	
	  	//cluster correspondence methods (Global)
	  	mth->calculateCorrespondenceCentroid(ca->centroid_collection,cb->centroid_collection,mp,0.1);
	  	
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
			}
			else
			{
				marker_pub.publish(mark_cluster(ca->clusters[(*mp)[j].index_query],id,"previous","bounding_box",rs,gs,bs));
				marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id,"current","bounding_box",rs,gs,bs));
			}
			id++;
		}*/

		/*pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_t(new pcl::PointCloud<pcl::PointXYZI>),rotated_p(new pcl::PointCloud<pcl::PointXYZI>);
		*keypoints_t = *ca->cloud;
		pcl_ros::transformPointCloud(*keypoints_t,*ca->cloud,t);

		ca->computeClusters(0.15,"single_cluster");
		cb->computeClusters(0.15,"single_cluster");

		pcl::toPCLPointCloud2(*ca->cluster_collection,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "previous";
		pub.publish(output);
		pcl::toPCLPointCloud2(*cb->cluster_collection,*cloud);
		pcl_conversions::fromPCL(*cloud, output);
		output.header.frame_id = "current";
		pub.publish(output);
  		
	  	pcl::CorrespondencesPtr mp(new pcl::Correspondences());
	  	
	  	//cluster correspondence methods (Global)
	  	mth->calculateCorrespondenceCentroid(ca->centroid_collection,cb->centroid_collection,mp,0.1);
	  	
	  	tf::Transform rot;
		rot.setIdentity();
		rot.setRotation(t.getRotation());
		pcl_ros::transformPointCloud(*keypoints_t,*rotated_p,rot);

	  	//moving object detection methods (Local)
	  	vector<double> param_vec = mth->getDirectionCloudVariance(rotated_p,ca->cluster_indices,ca->clusters,cb->clusters,mp);

	  	float rs=0.4,gs=0.6,bs=0.8,rd=0.8,gd=0.1,bd=0.4;int id = 1;
	  	for(int j=0;j<mp->size();j++)
		{
			cout<<"{"<<(*mp)[j].index_query<<"->"<<(*mp)[j].index_match<<"} Fit Score: "<<(*mp)[j].distance<<" Moving_Score: "<<param_vec[id-1]<<endl;
			//long threshold = (ca->clusters[(*mp)[j].index_query]->points.size()+cb->clusters[(*mp)[j].index_match]->points.size())/15;
			double threshold = 500.0;
			if(param_vec[id-1]>threshold)
			{
				marker_pub.publish(mark_cluster(ca->clusters[(*mp)[j].index_query],id,"previous","bounding_box",rd,gd,bd));
				marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id,"current","bounding_box",rd,gd,bd));
			}
			else
			{
				marker_pub.publish(mark_cluster(ca->clusters[(*mp)[j].index_query],id,"previous","bounding_box",rs,gs,bs));
				marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id,"current","bounding_box",rs,gs,bs));
			}
			id++;
		}*/