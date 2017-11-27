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
#include "std_msgs/Float32MultiArray.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>

typedef std::pair<std::string, std::vector<float> > vfh_model;
typedef pcl::VFHSignature308 PointT;

class recog
{
public:	
	ros::NodeHandle& nodeHandle_;

  	// ros::Subscriber PCsub_;

  	// ros::Publisher shape_pub_;

  	// ros::Publisher pub_shape_probabilities;

  	std::vector<ros::Publisher> pub_prob_vec;
  	//std::vector<ros::Publisher> pub_result_vec;

  	std::vector<ros::Subscriber> sub_cloud_vec;


	int maxcluster;
	int maxsample; 
	std::string modname;
	std::string message;
	std_msgs::Float32MultiArray shape_probabilities;

	recog(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
	{
		// if (!readParameters()) {
	 //    ROS_ERROR("Could not read parameters.");
	 //    ros::requestShutdown();
	 //  

	  maxcluster=3;
	  for (int i =0;i<maxcluster;i++)
	  {

	  	std::string subName = "/pcl_tut/cluster" + boost::lexical_cast<std::string>(i);
	  	std::string probName = "/Object_detection/shape_prob/cluster" + boost::lexical_cast<std::string>(i);
	  	ros::Subscriber sub_t;

	  	if (i==0) sub_t=nodeHandle_.subscribe(subName,1,&recog::recogCB0,this);
	  	if (i==1) sub_t=nodeHandle_.subscribe(subName,1,&recog::recogCB1,this);
	  	if (i==2) sub_t=nodeHandle_.subscribe(subName,1,&recog::recogCB2,this);

	  	sub_cloud_vec.push_back(sub_t);

	  	ros::Publisher pub = nodeHandle_.advertise<std_msgs::Float32MultiArray> (probName, 1);

        pub_prob_vec.push_back(pub);
	  }
	  
	  // shape_pub_ =
   //  	nodeHandle_.advertise<std_msgs::String>("/Object_detection/shape", 1);

   //  	pub_shape_probabilities = nodeHandle_.advertise<std_msgs::Float32MultiArray>("/object_classification/shape_probabilities", 1);
	  ROS_INFO("Successfully launchednode different cb.");



	}
	virtual ~recog()
	{
	}

	
	// bool readParameters()
	// {
 //    	if (!nodeHandle_.getParam("modname",
 //                            modname))
 //    		return false;

 //    	return true;


	// }

	void recogCB0(const sensor_msgs::PointCloud2ConstPtr& input)
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
			//pcl::PointCloud<pcl::VFHSignature308> vfhs;
			  // Compute the features
			vfh.compute (*vfhs);
			//ROS_INFO("vfhs loaded,  compare, no pointer");

			compare(*vfhs);
			// std_msgs::String msg;
			// msg.data=message;
			// shape_pub_.publish(msg);
			pub_prob_vec[0].publish(shape_probabilities);
			ROS_INFO("end of cb0 ");
	}

	void recogCB1(const sensor_msgs::PointCloud2ConstPtr& input)
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
			//pcl::PointCloud<pcl::VFHSignature308> vfhs;
			  // Compute the features
			vfh.compute (*vfhs);
			//ROS_INFO("vfhs loaded,  compare, no pointer");

			compare(*vfhs);
			// std_msgs::String msg;
			// msg.data=message;
			// shape_pub_.publish(msg);
			pub_prob_vec[1].publish(shape_probabilities);
			ROS_INFO("end of cb1 ");
	}

	void recogCB2(const sensor_msgs::PointCloud2ConstPtr& input)
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
			//pcl::PointCloud<pcl::VFHSignature308> vfhs;
			  // Compute the features
			vfh.compute (*vfhs);
			//ROS_INFO("vfhs loaded,  compare, no pointer");

			compare(*vfhs);
			// std_msgs::String msg;
			// msg.data=message;
			// shape_pub_.publish(msg);
			pub_prob_vec[2].publish(shape_probabilities);
			ROS_INFO("end of cb2 ");
	}

	bool
	loadHist (const pcl::PointCloud<pcl::VFHSignature308> &point, vfh_model &vfh)
	{
	  int vfh_idx;
	  //ROS_INFO("Loadhist");
	  // // Load the file as a PCD
	  // try
	  // {
	  //   pcl::PCLPointCloud2 cloud;
	  //   int version;
	  //   Eigen::Vector4f origin;
	  //   Eigen::Quaternionf orientation;
	  //   pcl::PCDReader r;
	  //   int type; unsigned int idx;
	  //   r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

	  //   vfh_idx = pcl::getFieldIndex (cloud, "vfh");
	  //   if (vfh_idx == -1)
	  //     return (false);
	  //   if ((int)cloud.width * cloud.height != 1)
	  //     return (false);
	  // }
	  // catch (const pcl::InvalidConversionException&)
	  // {
	  //   return (false);
	  // }

	  // Treat the VFH signature as a single Point Cloud

	  vfh.second.resize (308);


	  sensor_msgs::PointCloud2 c2;
  	  pcl::toROSMsg(point, c2);

	  vfh_idx = pcl::getFieldIndex (c2, "vfh");

	  std::vector <pcl::PCLPointField> fields;
	  getFieldIndex (point, "vfh", fields);

	  for (size_t i = 0; i < fields[vfh_idx].count; ++i)
	  {
	    vfh.second[i] = point.points[0].histogram[i];
	  }
	  vfh.first = "object";
	  return (true);
	}

	inline void
	nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
	                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
	{
	  // Query point
	  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
	  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

	  indices = flann::Matrix<int>(new int[k], 1, k);
	  distances = flann::Matrix<float>(new float[k], 1, k);
	  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
	  //delete[] p.ptr ();
	}

	/** \brief Load the list of file model names from an ASCII file
	  * \param models the resultant list of model name
	  * \param filename the input file name
	  */
	bool
	loadFileList (std::vector<vfh_model> &models, const std::string &filename)
	{
	  ifstream fs;
	  fs.open (filename.c_str ());
	  if (!fs.is_open () || fs.fail ())
	    return (false);

	  std::string line;
	  while (!fs.eof ())
	  {
	    getline (fs, line);
	    if (line.empty ())
	      continue;
	    vfh_model m;
	    m.first = line;
	    models.push_back (m);
	  }
	  fs.close ();
	  return (true);
	}

	void compare(const pcl::PointCloud<pcl::VFHSignature308> &point)
	{
		int k =20;
		int shapesize=7;

  		double thresh = 50;     // No threshold, disabled by default
  		std::string extension (".pcd");
  		transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

  		vfh_model histogram;
  		//ROS_INFO("NO load");
		loadHist (point, histogram);

		std::string kdtree_idx_file_name = "/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/kdtree.idx";
		std::string training_data_h5_file_name = "/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/training_data.h5";
		std::string training_data_list_file_name = "/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/training_data.list";

		std::vector<vfh_model> models;
		  flann::Matrix<int> k_indices;
		  flann::Matrix<float> k_distances;
		  flann::Matrix<float> data;
		  // Check if the data has already been saved to disk
		    loadFileList (models, training_data_list_file_name);
		    flann::load_from_file (data, training_data_h5_file_name, "training_data");
		    pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
		        (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
		  

		  // Check if the tree index has already been saved to disk
		    //ROS_INFO("No flann");
		 
		    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
		    index.buildIndex ();
		    nearestKSearch (index, histogram, k, k_indices, k_distances);

		  // Output the results on screen
		  pcl::console::print_highlight ("The closest %d neighbors are:\n", k);
		  double vote[7]={0,0,0,0,0,0,0};
		  //cube, holcube,sphere,holtriangle,holcylinder,holcross,star
		  for (int i = 0; i < k; ++i)
		  {
		  	pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
		        i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
		        //ROS_INFO("in1");	  
		  	//if (k_distances[0][i]>150) continue;
		  	std::string name=models.at (k_indices[0][i]).first;
			std::size_t found = name.find("Cube");
			//ROS_INFO("in2");
  			if (found!=std::string::npos) vote[0]+=1/(k_distances[0][i]);


  			found = name.find("Holcu");
  			if (found!=std::string::npos) vote[1]+=1/(k_distances[0][i]);

  			found = name.find("Sphere");
  			if (found!=std::string::npos) vote[2]+=1/(k_distances[0][i]);

  			found = name.find("Holtri");
  			if (found!=std::string::npos) vote[3]+=1/(k_distances[0][i]);

  			found = name.find("Holcy");
  			if (found!=std::string::npos) vote[4]+=1/(k_distances[0][i]);

  			found = name.find("Holcro");
  			if (found!=std::string::npos) vote[5]+=1/(k_distances[0][i]);

  			found = name.find("Star");
  			if (found!=std::string::npos) vote[6]+=1/(k_distances[0][i]);
  			//pcl::console::print_info ("end\n");
		  }

		  //normalize
		  double dsum=0;
		  for (int i=0;i<shapesize;i++) dsum+=vote[i];
		  for (int i=0;i<shapesize;i++) vote[i]/=dsum;




		  int order=-1;
		  double max=0.3;
		  for (int i=0;i<shapesize;i++)
		  {
		  	if (vote[i]>max)
		  	{
		  		order=i;
		  		max=vote[i];
		  	}

		  }
		  if (order==-1) message="N/A";
		  if (order==0) message="cube";
		  else if(order==1) message="holcube";
		  else if(order==2) message="sphere";
		  else if(order==3) message="holtriangle";
		  else if(order==4) message="holcylinder";
		  else if(order==5) message="holcross";
		  else if(order==6) message="star";
		  // // delete[] k_indices.ptr();
		  // // delete[] k_distances.ptr();    
		  // // delete[] data.ptr();
		  //ROS_INFO("end of compare");

		  //std_msgs::Float32MultiArray shape_probabilities;
		  shape_probabilities.data.clear();
		  for (int itr1 = 0; itr1<shapesize; itr1 ++) shape_probabilities.data.push_back(vote[itr1]);

      	//pub_shape_probabilities.publish(shape_probabilities);
	}


	


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_recog");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(1);
  recog rg(nodeHandle);

  for (;;) {
      ros::spinOnce();
      loop_rate.sleep();

  }
  ros::spin();
  return 0;
}




