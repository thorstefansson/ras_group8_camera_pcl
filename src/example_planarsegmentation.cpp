#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h> //<pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//for converting from rgb to hsv:
//#include <pcl/point_types_conversion.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//new:
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

//for printing:
#include <stdio.h>
#include<iostream>
#include <stdlib.h>

//for publicing center as multiarray: ,... maybe not all needed
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include "geometry_msgs/PointStamped.h"

using namespace std;

typedef pcl::PointXYZRGBA PointT;


ros::Publisher pub;
ros::Publisher pub_cloud;
// ros::Publisher pub;
std::vector<ros::Publisher> pub_vec;

//NEW:
ros::Publisher pub_center_point;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{



  //NEW PROGRAM:
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_outliers;
  pcl::PointCloud<pcl::PointXYZ> cloud_inliers;

  pcl::fromROSMsg (*input, cloud);

  //Instead of:
  //pcl::ModelCoefficients coefficients;
  //pcl::PointIndices inliers;
  //Try:
  pcl :: ModelCoefficients :: Ptr
  coefficients (new
  pcl :: ModelCoefficients  ());

  pcl :: PointIndices :: Ptr  inliers
  (new pcl :: PointIndices  ());



  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;


  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);

  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());

  //Instead of
  //seg.segment (inliers, coefficients);
  //Try:
  seg.segment (*inliers, *coefficients);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;


  //  Extract the inliers
  extract .setInputCloud (cloud.makeShared());
  extract . setIndices ( inliers );
  extract .setNegative (false );
  extract . filter (cloud_inliers);
  //ROS_INFO("%i",cloud_inliers.height*cloud_inliers.width);
  //std :: cerr
  //<<"PointCloud representing the planar component: "
  //<<cloud_inliers.width *


  // Extract outliers
  extract.setNegative (true);				// Extract the outliers
  extract.filter (cloud_outliers);		// cloud_outliers contains everything but the plane
  //ROS_INFO("%i",cloud_outliers.height*cloud_outliers.width);


  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  //Instead of
  //pcl_conversions::fromPCL(coefficients, ros_coefficients);
  //Try:
  pcl_conversions::fromPCL(*coefficients, ros_coefficients);


  //trying:
  sensor_msgs::PointCloud2 filtered_cloud;



  //pcl_conversions::toROSMsg(cloud_outliers, filtered_cloud);
  pcl::toROSMsg(cloud_outliers, filtered_cloud);
  pub_cloud.publish(filtered_cloud);
  ROS_INFO("%i",filtered_cloud.height);

  pub.publish (ros_coefficients);



  // Attempt to do cluster extraction:

  //this works:
  pcl::PointCloud<pcl::PointXYZ> downsampled_XYZ;



  downsampled_XYZ = cloud_outliers;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); //tree; //

  tree->setInputCloud (downsampled_XYZ.makeShared());
  //tree->setInputCloud (downsampled_XYZ);

  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;


  //For trap use a higher tolerance..
  ec.setClusterTolerance (0.02); // 2cm

  //Battery typically larger than 120
  //often at least 250 when not very far away, up to around 5-600
  //if lying down, floor can be a smaller plane.
  ec.setMinClusterSize (60);

  //Maximum size for colored objects to pick up around 160 points.
  ec.setMaxClusterSize (500);  //25000 original
  ec.setSearchMethod (tree);
  ec.setInputCloud (downsampled_XYZ.makeShared());

  //ec.setInputCloud (downsampled_XYZ);
  ec.extract (cluster_indices);

  ros::NodeHandle nh;


  //cout <<"hello"<< endl;
  //Create a publisher for each cluster
  for (int i = 0; i < cluster_indices.size(); ++i)
  {
      std::string topicName = "/pcl_tut/cluster" + boost::lexical_cast<std::string>(i);


      ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> (topicName, 1);

      pub_vec.push_back(pub);
  }

  int j = 0;
  int count=0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (downsampled_XYZ.points[*pit]); //*




      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      double x=0;
      double y=0;
      double z=0;
      pcl::PointXYZ point;

      int size = cloud_cluster->points.size () ;
      for (int i=0;i<cloud_cluster->points.size () ; i++)
      {
        point = cloud_cluster->at(i);//points.at[0,halfsize];//[1,halfsize];//0, cloud_cluster->points.size () / 2];//.at(0,20);//.at<pcl:>(y,x);

        x +=point.x;
        y += point.y;
        z += point.z;

        /*
        x =point.x;

        y = point.y;
        z = point.z;
*/
        //std::cout <<"x = " << x <<"  " << "y = " << y <<"  "<<"z = " << z << std::endl;

      }
        //this printing is not working....
      //std::cout <<"point = " << point << std::endl;

      x = x/size;
      y= y/size;
      z=z/size;
      std::cout <<"x = " << x <<"  " << "y = " << y <<"  "<<"z = " << z << std::endl;
      std::cout <<"size = " << size << std::endl;

      if (count==0){
      std::cout<<"startpublished"<<std::endl;
      geometry_msgs::PointStamped msg;
      msg.header.frame_id="/camera_depth_optical_frame";
      msg.header.stamp=ros::Time::now();
      msg.point.x=x;
      msg.point.y=y;
      msg.point.z=z;

      pub_center_point.publish(msg);
      std::cout<<"published"<<std::endl;
      }


      // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      //Convert the pointcloud to be used in ROS
      sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);

      pcl::toROSMsg (*cloud_cluster, *output);
      output->header.frame_id = input->header.frame_id;

      // Publish the data
      pub_vec[j].publish (output);
      //ROS_INFO("%i", *output.height);

      ++j;
      count++;
          }


}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb); //("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Create a ROS publisher for the output point cloud
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);

  //Create publisher for publishing center point:
  //pub_center_point = nh.advertise<pcl::PointXYZ> ("pcl_point", 1);
  pub_center_point = nh.advertise<geometry_msgs::PointStamped>("/Object_detection/position", 1);



  // Spin
  ros::spin ();
}
