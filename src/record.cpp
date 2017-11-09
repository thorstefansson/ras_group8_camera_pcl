#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h> //<pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>
#include "string"

class recordPC
{
public:	
	ros::NodeHandle& nodeHandle_;

  	ros::Subscriber PCsub_;

  	

	int count;
	int maxsample;
	std::string modname;

	recordPC(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
	{
		// if (!readParameters()) {
	 //    ROS_ERROR("Could not read parameters.");
	 //    ros::requestShutdown();
	 //  }
		maxsample=20;
		modname="Cube";
	  count=0;
	  PCsub_=nodeHandle_.subscribe("/pcl_tut/cluster0",1,&recordPC::recordCB,this);
	  ROS_INFO("Successfully launchednode.");



	}
	virtual ~recordPC()
	{
	}

	
	// bool readParameters()
	// {
	// 	if (!nodeHandle_.getParam("maxsample",
 //                            maxsample))
 //    		return false;
 //    	if (!nodeHandle_.getParam("modname",
 //                            modname))
 //    		return false;

 //    	return true;


	// }

	void recordCB(const sensor_msgs::PointCloud2ConstPtr& input)
	{
		if(count<maxsample)
		{
			//pcl::PCDWriter writer;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			//pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::fromROSMsg (*input, *cloud);

			//search fo normals

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  			ne.setInputCloud (cloud);
  			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  			ne.setSearchMethod (tree);

			pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
			ne.setRadiusSearch (0.03);

		  	// Compute the features
		  	ne.compute (*normals);

			pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
			vfh.setInputCloud (cloud);
			vfh.setInputNormals (normals);
			vfh.setSearchMethod (tree);
			  // Output datasets
			pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

			  // Compute the features
			vfh.compute (*vfhs);



			std::stringstream ss;
			ss<<"cloud_cluster_"<<modname<<"_"<<count<<".pcd";
			std::stringstream vss;
			vss<<"cloud_cluster_"<<modname<<"_"<<count<<"_vfh"<<".pcd";
			pcl::io::savePCDFileASCII (ss.str(), *cloud);
			pcl::io::savePCDFileASCII (vss.str(), *vfhs);
			//writer.write<pcl::PointXYZ>(ss.str(),cloud,false);
			//printf(modname.c_str());
			// for (size_t i = 0; i < cloud.points.size (); ++i)
   //  			std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
			ROS_INFO("record success ,%d",count);
			count++;

		}
		

	}


	


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_PC_record");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(1.0);
  recordPC rd(nodeHandle);

  for (;;) {
      ros::spinOnce();
      loop_rate.sleep();

  }
  ros::spin();
  return 0;
}
