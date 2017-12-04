#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h> //<pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>

//for converting from rgb to hsv:
//#include <pcl/point_types_conversion.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
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

//to read and write file:
#include <fstream>
#include <string>

using namespace std;

//typedef pcl::PointXYZRGBA PointT;


ros::Publisher pub;
ros::Publisher pub_cloud;
// ros::Publisher pub;
std::vector<ros::Publisher> pub_vec;

//NEW:
ros::Publisher pub_center_point;


//for testing
ros::Publisher pub_test_cloud;


// Initializations needed for removing walls from lower cloud:
pcl::PointCloud<pcl::PointXYZHSV>::Ptr lower_cloud_no_floor;
pcl::IndicesPtr lower_cloud_wall_inliers(new std::vector<int>);
pcl::SampleConsensusModelPlane<pcl::PointXYZHSV>::Ptr model;
//pcl::PointCloud<pcl::PointXYZHSV> cloud_no_wall_outliers;
pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_no_wall_outliers (new pcl::PointCloud<pcl::PointXYZHSV>);
pcl::ExtractIndices<pcl::PointXYZHSV> extract2;
pcl::PointCloud<pcl::PointXYZHSV> lower_cloud_walls;



using namespace pcl;

void
  PointXYZRGBtoXYZHSV (const PointXYZRGB& in,
                       PointXYZHSV&       out)
  {
    const unsigned char max = std::max (in.r, std::max (in.g, in.b));
    const unsigned char min = std::min (in.r, std::min (in.g, in.b));

    out.x = in.x; out.y = in.y; out.z = in.z;
    out.v = static_cast <float> (max) / 255.f;

    if (max == 0) // division by zero
    {
      out.s = 0.f;
      out.h = 0.f; // h = -1.f;
      return;
    }

    const float diff = static_cast <float> (max - min);
    out.s = diff / static_cast <float> (max);

    if (min == max) // diff == 0 -> division by zero
    {
      out.h = 0;
      return;
    }

    if      (max == in.r) out.h = 60.f * (      static_cast <float> (in.g - in.b) / diff);
    else if (max == in.g) out.h = 60.f * (2.f + static_cast <float> (in.b - in.r) / diff);
    else                  out.h = 60.f * (4.f + static_cast <float> (in.r - in.g) / diff); // max == b

    if (out.h < 0.f) out.h += 360.f;
}

void
 PointCloudXYZRGBtoXYZHSV (const PointCloud<PointXYZRGB>& in,
                           PointCloud<PointXYZHSV>&       out)
 {
   out.width   = in.width;
   out.height  = in.height;
   for (size_t i = 0; i < in.points.size (); i++)
   {
     PointXYZHSV p;
     PointXYZRGBtoXYZHSV (in.points[i], p);
     out.points.push_back (p);
   }
}


template <typename T>
string ToString(T val)
{
    stringstream stream;
    stream << val;
    return stream.str();
}
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{


    cout << "begin" << endl;

    //PASS THROUGH FILTER:
    // divides the point cloud into upper and lower cloud

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
    pcl::PCLPointCloud2 cloud_filtered_upper;
    pcl::PCLPointCloud2 cloud_filtered_lower;

    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud2);

    // Perform the actual filtering
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud (cloudPtr);//(cloudPtr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.05, 1);
    pass.filter (cloud_filtered_lower);

    pass.setFilterLimitsNegative(true);
    pass.filter(cloud_filtered_upper);

    /*
  //NEW PROGRAM:
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_outliers;
  pcl::PointCloud<pcl::PointXYZ> cloud_inliers;
  */

//  //TRY TO ADD RGB VALUES TO CLOUD:
//  pcl::PointCloud<pcl::PointXYZRGB> cloud;
//  pcl::PointCloud<pcl::PointXYZRGB> cloud_outliers;
//  pcl::PointCloud<pcl::PointXYZRGB> cloud_inliers;
//    pcl::PointCloud<pcl::PointXYZRGB> RGBcloud;
//  //pcl::fromROSMsg (*input, cloud);


  //TRY TO GET HSV VALUES:
  pcl::PointCloud<pcl::PointXYZRGB> RGBcloud;
  //pcl::fromROSMsg (*input, RGBcloud);

  pcl::fromPCLPointCloud2(cloud_filtered_lower, RGBcloud);

  pcl::PointCloud<pcl::PointXYZHSV> cloud;
  pcl::PointCloud<pcl::PointXYZHSV> cloud_outliers;
  pcl::PointCloud<pcl::PointXYZHSV> cloud_inliers;

  //pcl::PointCloud<pcl::PointXYZHSV>::cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
  PointCloudXYZRGBtoXYZHSV(RGBcloud, cloud); // RGB -> HSV

  /*
  pcl::PointXYZRGB point1;
  pcl::PointXYZHSV point2;
  point1 = RGBcloud.at(10);
  point2 = cloud.at(10);
  std::cout <<"RGB "<<  "x = " << point1.x <<"  " << "y = " << point1.y <<"  "<<"z = " << point1.z << std::endl;
  std::cout <<"HSV "<<  "x = " << point2.x <<"  " << "y = " << point2.y <<"  "<<"z = " << point2.z << std::endl;
  //std::cout <<"RGB cloud size" << RGBcloud.size() << std::endl;
  //std::cout <<"HSV cloud size" << cloud.size() << std::endl;*/





  // Instead of the following code segmenting out largest plane:
  /*
  //FILTERING OUT LARGEST PLANE:
  pcl :: ModelCoefficients :: Ptr
  coefficients (new
  pcl :: ModelCoefficients  ());

  pcl :: PointIndices :: Ptr  inliers
  (new pcl :: PointIndices  ());

  // Create the segmentation object
  //pcl::SACSegmentation<pcl::PointXYZ> seg;
  //RGB VALS:
//  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  //HSV values:
  pcl::SACSegmentation<pcl::PointXYZHSV> seg;

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);
  // Create the filtering object
  //pcl::ExtractIndices<pcl::PointXYZ> extract;
  //WITH RGB:
//  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  //HSV:
  pcl::ExtractIndices<pcl::PointXYZHSV> extract;
  //  Extract the inliers
  extract .setInputCloud (cloud.makeShared());
  extract . setIndices ( inliers );
  extract .setNegative (false );
  extract . filter (cloud_inliers);
  //ROS_INFO("%i",cloud_inliers.height*cloud_inliers.width);
  // Extract outliers
  extract.setNegative (true);				// Extract the outliers
  extract.filter (cloud_outliers);		// cloud_outliers contains everything but the plane !!!!!!!
  //ROS_INFO("%i",cloud_outliers.height*cloud_outliers.width);
  */

  // Try to remove the floor given its coefficients:
  //these are the coefficients of the floor: values: [-0.04023124650120735, -0.8639819622039795, -0.5019128918647766, 0.12330468744039536]

  pcl::SampleConsensusModelPlane<pcl::PointXYZHSV>::Ptr floor_model;
  pcl::IndicesPtr lower_cloud_floor_inliers(new std::vector<int>);

  Eigen::VectorXf floor_coefficients(4);

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr lower_cloud = cloud.makeShared();

  floor_model.reset(new pcl::SampleConsensusModelPlane<pcl::PointXYZHSV>(lower_cloud));

  float floor_coefficients_array [4] = {-0.04023124650120735, -0.8639819622039795, -0.5019128918647766, 0.12330468744039536};

  for (int i = 0; i<4 ; i++){
      floor_coefficients[i] =floor_coefficients_array[i];
  }

  floor_model->selectWithinDistance(floor_coefficients, 0.01, *lower_cloud_floor_inliers);


  // Create the filtering object
  //pcl::ExtractIndices<pcl::PointXYZ> extract;
  //WITH RGB:
//  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  //HSV:
  pcl::ExtractIndices<pcl::PointXYZHSV> extract;
  //  Extract the inliers
  extract .setInputCloud (lower_cloud);
  extract . setIndices ( lower_cloud_floor_inliers );
  extract .setNegative (false );
  extract . filter (cloud_inliers);
  //ROS_INFO("%i",cloud_inliers.height*cloud_inliers.width);
  // Extract outliers
  extract.setNegative (true);				// Extract the outliers
  extract.filter (cloud_outliers);		// cloud_outliers contains everything but the plane !!!!!!!
  //ROS_INFO("%i",cloud_outliers.height*cloud_outliers.width);


  //SEGMENT OUT PLANES FROM THE UPPER CLOUD: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
  //TRY TO GET HSV VALUES:
  pcl::PointCloud<pcl::PointXYZRGB> RGB_upper_cloud;

  pcl::fromPCLPointCloud2(cloud_filtered_upper, RGB_upper_cloud);

  pcl::PointCloud<pcl::PointXYZHSV> upper_HSV_cloud;
  //pcl::PointCloud<pcl::PointXYZHSV> cloud_wall_inliers;
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr upper_cloud_no_walls (new pcl::PointCloud<pcl::PointXYZHSV>);

  PointCloudXYZRGBtoXYZHSV(RGB_upper_cloud, upper_HSV_cloud); // RGB -> HSV
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr upper_HSV_cloudPtr = upper_HSV_cloud.makeShared();

  // Initializations needed for removing walls from lower cloud:
  lower_cloud_no_floor = cloud_outliers.makeShared();

  //FILTERING OUT LARGEST PLANE:
  pcl :: ModelCoefficients :: Ptr wall_coefficients (new pcl :: ModelCoefficients  ());
  pcl :: PointIndices :: Ptr  wall_inliers (new pcl :: PointIndices  ());

  //START LOOP FOR MULTIPLE WALLS:
  // Create the segmentation object
  //pcl::SACSegmentation<pcl::PointXYZ> seg;
  //RGB VALS:
//  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  //HSV values:
  pcl::SACSegmentation<pcl::PointXYZHSV> seg2;

  // Optional
  seg2.setOptimizeCoefficients (true);
  // Mandatory
  seg2.setModelType (pcl::SACMODEL_PLANE);
  seg2.setMethodType (pcl::SAC_RANSAC);
  seg2.setMaxIterations (100);
  seg2.setDistanceThreshold (0.01);


  int iteration = 0, nr_points_upper_cloud = (int) upper_HSV_cloudPtr -> points.size();
  //START LOOP HERE:

  while(upper_HSV_cloudPtr -> points.size() > 50 && iteration < 6 ) //0.1 * nr_points_upper_cloud && iteration < 6 )
  {
  seg2.setInputCloud (upper_HSV_cloudPtr);
  seg2.segment (*wall_inliers, *wall_coefficients);

  if (wall_inliers->indices.size () == 0)
      {
        cout << "Could not estimate a planar model for the given dataset." << endl;
        break;
      }

  cout << "size of inliers: " << wall_inliers->indices.size () << endl;
  cout << "size of remaining point cloud: " << upper_HSV_cloudPtr -> points.size() << endl;
  //for testing, see inliers of extracted plane, upper cloud:


  pcl::ExtractIndices<pcl::PointXYZHSV> extract_plane_upper;
  // Extract the inliers
  extract_plane_upper .setInputCloud (upper_HSV_cloudPtr);
  extract_plane_upper . setIndices ( wall_inliers );
  extract_plane_upper .setNegative (true );         // Extract the outliers
  extract_plane_upper . filter (*upper_cloud_no_walls);
  /*
  extract_plane_upper .setNegative (false );
  extract_plane_upper . filter (cloud_wall_inliers);*/
  cout << "done extracting from upper cloud" << endl;
  //ROS_INFO("%i",cloud_inliers.height*cloud_inliers.width);
  //instead of:
  upper_HSV_cloudPtr.swap(upper_cloud_no_walls);
  //try:
  //upper_HSV_cloudPtr=upper_cloud_no_walls;


//REMOVE THE SAME PLANE FROM LOWER CLOUD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  //cout << "coefficients:" << *wall_coefficients <<endl;

  model.reset(new pcl::SampleConsensusModelPlane<pcl::PointXYZHSV>(lower_cloud_no_floor));

  Eigen::VectorXf model_coefficients(4);

  for (int i = 0; i<4 ; i++){
      model_coefficients[i] =wall_coefficients->values[i];
  }

  cout << "number of inliers: " << model->countWithinDistance(model_coefficients, 0.01) <<endl;

  model->selectWithinDistance(model_coefficients, 0.02, *lower_cloud_wall_inliers);

  //HSV:
  //  Extract the inliers
  extract2 .setInputCloud (lower_cloud_no_floor);
  extract2 . setIndices ( lower_cloud_wall_inliers);//inliers2 );
  //cout <<"inliers:" << lower_cloud_wall_inliers << endl;
  extract2.setNegative (false);				// Extract the inliers
  /*extract2.filter (lower_cloud_walls);		// Walls belonging to lower cloud !!!!!!!
  cout <<"inliers:" << lower_cloud_walls.width*lower_cloud_walls.height << endl;*/
  // Extract outliers
  extract2.setNegative (true);				// Extract the outliers
  extract2.filter (*cloud_no_wall_outliers);		// cloud_outliers contains everything but the plane !!!!!!!
  //cout <<"hi" << endl;
  cout <<"lower cloud without wall:" << cloud_no_wall_outliers->width * cloud_no_wall_outliers->height << endl;

  //instead of:
  lower_cloud_no_floor.swap(cloud_no_wall_outliers);
  //try:
  //lower_cloud_no_floor=cloud_no_wall_outliers;

  iteration++;

  //I get error:
  /*
    lower cloud without wall:334
[pcl::ExtractIndices::applyFilter] The indices size exceeds the size of the input.
coefficients:header:
*/

}  //END LOOP

  cout << "iterations: " << iteration << endl;

  //This only works when calculating largest plane:
  /*
  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(*coefficients, ros_coefficients);
  pub.publish (ros_coefficients);*/


  sensor_msgs::PointCloud2 filtered_cloud;
  //pcl_conversions::toROSMsg(cloud_outliers, filtered_cloud);
  pcl::toROSMsg(*lower_cloud_no_floor, filtered_cloud);
  filtered_cloud.header.frame_id = input ->header.frame_id;
  pub_cloud.publish(filtered_cloud);
  //ROS_INFO("%i",filtered_cloud.width);




  //for testing:
  //sensor_msgs::PointCloud2 test_upper_cloud;
  //pcl_conversions::toROSMsg(cloud_outliers, filtered_cloud);
  /*pcl::toROSMsg(cloud_wall_inliers, test_upper_cloud);
  test_upper_cloud.header.frame_id = input ->header.frame_id;
  pub_test_cloud.publish(test_upper_cloud);
  //ROS_INFO("%i",filtered_cloud.width);
*/

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

  // for testing
  pub_test_cloud = nh.advertise<sensor_msgs::PointCloud2> ("test_cloud", 1);

  //Create publisher for publishing center point:
  //pub_center_point = nh.advertise<pcl::PointXYZ> ("pcl_point", 1);
  pub_center_point = nh.advertise<geometry_msgs::PointStamped>("/Object_detection/position", 1);


  //Slower loop rate perhaps when training:
  ros::Rate loop_rate(2);
  for (;;) {
      ros::spinOnce();
      loop_rate.sleep();
  }


  // Spin
  ros::spin ();
}
