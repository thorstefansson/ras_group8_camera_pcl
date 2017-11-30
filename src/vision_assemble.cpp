#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>

//for cluster extraction:
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/lexical_cast.hpp>


//for publicing center as multiarray: ,... maybe not all needed
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include "geometry_msgs/PointStamped.h"

//to read and write file:
#include <fstream>
#include <string>

using namespace std;

class vision
{
public:
	ros::NodeHandle& nodeHandle_;
	ros::Publisher pub;

	vision(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
	{
	}

	sensor_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
		pcl_conversions::toPCL(*cloud_msg, *cloud);
		sensor_msgs::PointCloud2  voxcloud=preprocess(cloud);




	}


	sensor_msgs::PointCloud2 preprocess(pcl::PCLPointCloud2* cloud)
	{
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		pcl::PCLPointCloud2 cloud_filtered;
		pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (cloudPtr);
		sor.setMeanK(30);
		sor.setStddevMulThresh(2);
		sor.filter(cloud_filtered);
		//return(cloud_filtered)

		pcl::PCLPointCloud2ConstPtr cloudPtr2=cloud_filtered.getinputcloud();
		pcl::PCLPointCloud2 cloud_filtered2;
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor2;
		sor2.setInputCloud (cloudPtr2);//(cloudPtr);
		  //Voxel Grid Filter
		sor2.setLeafSize (0.005, 0.005, 0.005);
		sor2.filter (cloud_filtered2);

		sensor_msgs::PointCloud2 output;
  		pcl_conversions::moveFromPCL(cloud_filtered2, output);
  		return(output);
  	}

	






};