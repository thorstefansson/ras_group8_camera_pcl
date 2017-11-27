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
#include <std_msgs/String.h>

class recordPC
{
public:	
	ros::NodeHandle& nodeHandle_;

  	ros::Subscriber PCsub_;
  	ros::Subscriber name_;

  	pcl::PointCloud<pcl::VFHSignature308>::Ptr body_vfhs ;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr body_cloud ;

  	

	int count;
	int maxsample;
	std::string modname;

	recordPC(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
	{
		// if (!readParameters()) {
	 //    ROS_ERROR("Could not read parameters.");
	 //    ros::requestShutdown();
	 //  }
		// maxsample=50;
		// modname="Holcu";
	  count=0;
	  PCsub_=nodeHandle_.subscribe("/pcl_tut/cluster0",1,&recordPC::recordCB,this);
	  name_=nodeHandle_.subscribe("/record_commend",1,&recordPC::nameCB,this);
	  body_vfhs= pcl::PointCloud<pcl::VFHSignature308>::Ptr(new pcl::PointCloud<pcl::VFHSignature308> ());
	  body_cloud= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);



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

	void nameCB(const std_msgs::StringPtr& input)
	{
		std::string classname=input->data;
		std::stringstream ss;
		ss<<"/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/data/"<<classname<<"/cloud_cluster_"<<classname<<"_"<<count<<".pcd";
		std::stringstream vss;
		vss<<"/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/data/"<<classname<<"/cloud_cluster_"<<classname<<"_"<<count<<"_vfh"<<".pcd";
		pcl::io::savePCDFileASCII (ss.str(), *body_cloud);
		pcl::io::savePCDFileASCII (vss.str(), *body_vfhs);
		//writer.write<pcl::PointXYZ>(ss.str(),cloud,false);
		//printf(modname.c_str());
		// for (size_t i = 0; i < cloud.points.size (); ++i)
//  			std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
		ROS_INFO("record success ,%d",count);
		count++;

	}

	void recordCB(const sensor_msgs::PointCloud2ConstPtr& input)
	{
		// if(count<maxsample)
		// {
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

			*body_vfhs=*vfhs;
			*body_cloud=*cloud;



			// std::stringstream ss;
			// ss<<"cloud_cluster_"<<modname<<"_"<<count<<".pcd";
			// std::stringstream vss;
			// vss<<"cloud_cluster_"<<modname<<"_"<<count<<"_vfh"<<".pcd";
			// pcl::io::savePCDFileASCII (ss.str(), *cloud);
			// pcl::io::savePCDFileASCII (vss.str(), *vfhs);
			// //writer.write<pcl::PointXYZ>(ss.str(),cloud,false);
			// //printf(modname.c_str());
			// // for (size_t i = 0; i < cloud.points.size (); ++i)
   // //  			std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
			// ROS_INFO("record success ,%d",count);
			//count++;

		//}
		

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
