#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/passthrough.h>

ros::Publisher pub_upper;
ros::Publisher pub_lower;
ros::Publisher pub_upper_right;
ros::Publisher pub_upper_left;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered_upper;
  pcl::PCLPointCloud2 cloud_filtered_lower;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);


  // Perform the actual filtering
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloudPtr);//(cloudPtr);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.05, 1);
  pass.filter (cloud_filtered_lower);

  pass.setFilterLimitsNegative(true);
  pass.filter(cloud_filtered_upper);


  /*
  //filter upper cloud into right and left part where walls typically are

  pcl::PCLPointCloud2* upper_cloud = &cloud_filtered_upper;
  pcl::PCLPointCloud2ConstPtr upperPtr(upper_cloud);

  //left
  pcl::PCLPointCloud2 cloud_filtered_upper_left;
  pcl::PassThrough<pcl::PCLPointCloud2> pass_left;
  pass_left.setInputCloud (upperPtr);//(cloudPtr);
  pass_left.setFilterFieldName("x");
  pass_left.setFilterLimits(FLT_MIN, -10.0);
  pass_left.filter (cloud_filtered_upper_left);

  //right
  pcl::PCLPointCloud2 cloud_filtered_upper_right;
  pcl::PassThrough<pcl::PCLPointCloud2> pass_right;
  pass_right.setInputCloud (upperPtr);//(cloudPtr);
  pass_right.setFilterFieldName("x");
  pass_right.setFilterLimits(10.0, FLT_MAX);
  pass_right.filter (cloud_filtered_upper_right);
*/

  // Convert to ROS data type
  //upper
  sensor_msgs::PointCloud2 output_upper;
  pcl_conversions::moveFromPCL(cloud_filtered_upper, output_upper);
//lower
  sensor_msgs::PointCloud2 output_lower;
  pcl_conversions::moveFromPCL(cloud_filtered_lower, output_lower);

  /*
  //upper left
  sensor_msgs::PointCloud2 output_upper_left;
  pcl_conversions::moveFromPCL(cloud_filtered_upper_left, output_upper_left);
  //upper
  sensor_msgs::PointCloud2 output_upper_right;
  pcl_conversions::moveFromPCL(cloud_filtered_upper_right, output_upper_right);

*/

  // Publish the data
  pub_upper.publish (output_upper);
  pub_lower.publish (output_lower);
  /*
  pub_upper_right.publish (output_upper_right);
  pub_upper_left.publish (output_upper_left);
*/

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "passthrough_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);//("/camera/depth/points", 1, cloud_cb);//

  // Create a ROS publisher for the output point cloud

  pub_upper = nh.advertise<sensor_msgs::PointCloud2> ("output_upper", 1);
  pub_lower = nh.advertise<sensor_msgs::PointCloud2> ("output_lower", 1);
  pub_upper_right = nh.advertise<sensor_msgs::PointCloud2> ("output_upper_right", 1);
  pub_upper_left = nh.advertise<sensor_msgs::PointCloud2> ("output_upper_left", 1);

  // Spin
  ros::spin ();
}
