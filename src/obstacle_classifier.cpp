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
#include "std_msgs/Float32MultiArray.h"

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

std::vector<ros::Publisher> prob_vec;

//NEW:
ros::Publisher pub_center_point;
//for many centers:
std::vector<ros::Publisher> pub_vec_centers;

//ros::Publisher pub_color_probabilities;



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

    cout << "beginning of callback" << endl;

    /*
  //NEW PROGRAM:
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_outliers;
  pcl::PointCloud<pcl::PointXYZ> cloud_inliers;
  */

  //TRY TO ADD RGB VALUES TO CLOUD:
  //pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //pcl::PointCloud<pcl::PointXYZRGB> cloud_outliers;
  //pcl::PointCloud<pcl::PointXYZRGB> cloud_inliers;
  //pcl::fromROSMsg (*input, cloud);

  //TRY TO GET HSV VALUES:
  pcl::PointCloud<pcl::PointXYZRGB> RGBcloud;
  pcl::fromROSMsg (*input, RGBcloud);

  pcl::PointCloud<pcl::PointXYZHSV> cloud;
  pcl::PointCloud<pcl::PointXYZHSV> cloud_outliers;
  pcl::PointCloud<pcl::PointXYZHSV> cloud_inliers;

  //std_msgs::Float32MultiArray color_probabilities;



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


  pcl :: ModelCoefficients :: Ptr coefficients (new pcl :: ModelCoefficients  ());

  pcl :: PointIndices :: Ptr  inliers  (new pcl :: PointIndices  ());



  // Create the segmentation object
  //pcl::SACSegmentation<pcl::PointXYZ> seg;

  //RGB VALS:
  //pcl::SACSegmentation<pcl::PointXYZRGB> seg;

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

  //Instead of
  //seg.segment (inliers, coefficients);
  //Try:
  seg.segment (*inliers, *coefficients);



  // Create the filtering object
  //pcl::ExtractIndices<pcl::PointXYZ> extract;
  //WITH RGB:
  //pcl::ExtractIndices<pcl::PointXYZ> extract;
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
  //pcl::PointCloud<pcl::PointXYZ> downsampled_XYZ;
  //WITH RGB:
  //pcl::PointCloud<pcl::PointXYZRGB> downsampled_XYZ;
  //HSV:
  pcl::PointCloud<pcl::PointXYZHSV> downsampled_XYZ;


  downsampled_XYZ = cloud_outliers;


  // Creating the KdTree object for the search method of the extraction
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); //tree; //
  //RGB:
  //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>); //tree; //
//HSV
  pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZHSV>);

  tree->setInputCloud (downsampled_XYZ.makeShared());
  //tree->setInputCloud (downsampled_XYZ);

  std::vector<pcl::PointIndices> cluster_indices;

  //pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//RGB:
  //pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
//HSV:
  pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> ec;

  //For trap use a higher tolerance..
  ec.setClusterTolerance (0.02); // 2cm

  //Battery typically larger than 120
  //often at least 250 when not very far away, up to around 5-600
  //if lying down, floor can be a smaller plane.
  ec.setMinClusterSize (120);

  //Maximum size for colored objects to pick up around 220 points.
  ec.setMaxClusterSize (3000);  //25000 original. can use 500 for objects..
  ec.setSearchMethod (tree);
  ec.setInputCloud (downsampled_XYZ.makeShared());

  //ec.setInputCloud (downsampled_XYZ);
  ec.extract (cluster_indices);


  ros::NodeHandle nh;


  //cout <<"hello"<< endl;
  //Create a publisher for each cluster
  int maxcluster=3;

  for (int i = 0; i < cluster_indices.size(); ++i)
  {
      std::string topicName = "/pcl/cluster/obstacle" + boost::lexical_cast<std::string>(i);


      ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> (topicName, 1);

      pub_vec.push_back(pub);

      /*
      std::string topicName2 = "/Object_detection/color_prob/cluster" + boost::lexical_cast<std::string>(i);

      ros::Publisher pub2 = nh.advertise<std_msgs::Float32MultiArray> (topicName2, 1);

      prob_vec.push_back(pub2);

      */

      //TRY TO PUBLISH ALL CENTER POINTS:

      topicName = "/pcl/cluster_center" + boost::lexical_cast<std::string>(i);

      pub_center_point = nh.advertise<geometry_msgs::PointStamped>(topicName, 1);

      pub_vec_centers.push_back(pub_center_point);
  }

  int j = 0;
  int count=0;

  double H;
  double S;
  double V;



  /*
  //Get classification matrix:
  int** result_matrix = new int*[255];
  ifstream resultFile;
  resultFile.open("/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera1/classifiers/result_matrix_colors_and_obstacles.dat");

  for(int lin=0;lin<255;lin++)
  {
      result_matrix[lin] = new int[360];

      for(int col=0;col<360;col++)
      {
          //resultFile >> result_matrix.at<int>(lin,col) ;
           //resultFile >> result_matrix[lin][col];//result_matrix.at<int>(lin,col) ;
          resultFile >> result_matrix[lin][col];
          //cout << result_matrix[lin][col];//result_matrix.at<int>(0,1);
          //cout <<result_matrix.at<int>(0,1) << " ";
      }

      //cout << endl;
  }
  resultFile.close();
*/



  float color_likelihood = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
      //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      //RGB:
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      //HSV:
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZHSV>);

      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (downsampled_XYZ.points[*pit]); //*


      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      double x=0;
      double y=0;
      double z=0;
      //float color_votes[9] = {0.0};
      //pcl::PointXYZ point;
      //RGB:
      //pcl::PointXYZRGB point;
      //HSV:
      pcl::PointXYZHSV point;


      int size = cloud_cluster->points.size () ;

      //if(size<400){
          for (int i=0;i<cloud_cluster->points.size () ; i++)
          {
              point = cloud_cluster->at(i);

              x +=point.x;
              y += point.y;
              z += point.z;

              H = point.h;
              S = point.s;
              S = S*254;
              V = point.v;

              //make if statements to calculate how many points are likely to belong to a valuable object:

              if((H<57 && S>183) || (H> 90 && H<200 && S > 140) || (H> 212 && H<260 && H> 36 && H<127) || (H>355 && S>212)){
                  color_likelihood++;
              }

              //color_votes[result_matrix[(int)S][(int)H]]++;

              //change this depending on color to remove outliers:
              // 115 for dark green, 130 for light green, 180 for light blue,20 for orange, 38 for yellow, 230 purple, 350-10 for red
              //if (count==0 && abs(115-H < 20)){
              //if (count==0){
              //write training data to files
              //Hfile << ToString(H) << endl;
              //Sfile << ToString(S) << endl;
              //}

              /*
        x =point.x;.
        y = point.y;
        z = point.z;
*/
              //std::cout <<"x = " << x <<"  " << "y = " << y <<"  "<<"z = " << z << std::endl;
              //std::cout << "H= " << H << " S= " << S << endl;
          }
      /*}
      else{
          for (int i=0;i<cloud_cluster->points.size () ; i++)
          {
              point = cloud_cluster->at(i);

              x +=point.x;
              y += point.y;
              z += point.z;
          }
      }*/


      x = x/size;
      y= y/size;
      z=z/size;

      color_likelihood /= size;

      cout << "likelihood of obstacle cluster " << j << " being a valuable object: " << color_likelihood << endl;

      //for(int itr=0; itr<9;itr++) color_votes[itr] /= size;

      //color_votes = color_votes/size;
/*
      std::cout << "orange:" << color_votes[0] << " red:" << color_votes[1] << " yellow:" << color_votes[2]
                   << " purple:" << color_votes[3] << " blue:" << color_votes[4] << " dark_green:" << color_votes[5]
                      <<" light green:" << color_votes[6] <<" battery:" << color_votes[7] << " trap:" << color_votes[8]
                        << endl;
*/
      //std::cout <<"x = " << x <<"  " << "y = " << y <<"  "<<"z = " << z << std::endl;
      std::cout <<"size = " << size << std::endl;

      //color_probabilities.data.clear();

      //if (count==0){
      //std::cout<<"startpublished"<<std::endl;
      geometry_msgs::PointStamped msg;
      //msg.header.frame_id="/camera_depth_optical_frame";
      msg.header.stamp=ros::Time::now();
      msg.point.x=x;
      msg.point.y=y;
      msg.point.z=z;

      pub_vec_centers[j].publish(msg);
      //pub_center_point.publish(msg);
    //}
      //if(count>=3) break;

      // std::cout << "orange:" << color_votes[0] << " red:" << color_votes[1] << " yellow:" << color_votes[2]
      //              << " purple:" << color_votes[3] << " blue:" << color_votes[4] << " dark_green:" << color_votes[5]
      //                 <<" light green:" << color_votes[6] <<" battery:" << color_votes[7] << " trap:" << color_votes[8]
      //                   << endl;
      // std::cout<<"published"<<std::endl;
      //for (int itr1 = 0; itr1<9; itr1 ++) color_probabilities.data.push_back(color_votes[itr1]);

      //prob_vec[count].publish(color_probabilities);

      //int x=point.x;
      //int y = point.y;
      //int z= point.z;
      //std::cout <<"x = " << x <<"  " << "y = " << y <<"  "<<"z = " << z << std::endl;

      // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      //Convert the pointcloud to be used in ROS
      sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);

      pcl::toROSMsg (*cloud_cluster, *output);
      output->header.frame_id = input->header.frame_id;

      // Publish the data
      pub_vec[j].publish (output);
      //ROS_INFO("%i", *output.height);
      cout << "published" << endl;

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

  //pub_color_probabilities = nh.advertise<std_msgs::Float32MultiArray>("/object_classification/color_probabilities", 1);


  //Slower loop rate perhaps when training:
  ros::Rate loop_rate(2);
  for (;;) {
      ros::spinOnce();
      loop_rate.sleep();
  }


  // Spin
  ros::spin ();
}
