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

  	ros::Subscriber PCsub_;

  	ros::Publisher shape_pub_;



  	

	int count;
	int maxsample;
	std::string modname;
	std::string message;

	recog(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
	{
		// if (!readParameters()) {
	 //    ROS_ERROR("Could not read parameters.");
	 //    ros::requestShutdown();
	 //  }
		maxsample=20;
		modname="Cube";
		message="";

	  count=0;
	  PCsub_=nodeHandle_.subscribe("/pcl_tut/cluster0",1,&recog::recogCB,this);
	  shape_pub_ =
    	nodeHandle_.advertise<std_msgs::String>("/Object_detection/shape", 1);
	  ROS_INFO("Successfully launchednode.");



	}
	// virtual ~recog()
	// {
	// }

	
	// bool readParameters()
	// {
 //    	if (!nodeHandle_.getParam("modname",
 //                            modname))
 //    		return false;

 //    	return true;


	// }

	void recogCB(const sensor_msgs::PointCloud2ConstPtr& input)
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
			count++;
			ROS_INFO("vfhs loaded,  compare, no pointer");

			compare(*vfhs);
			std_msgs::String msg;
			msg.data=message;
			shape_pub_.publish(msg);
			//ROS_INFO("end of cb ");
		




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
		int k = 4;

  		double thresh = 50;     // No threshold, disabled by default
  		std::string extension (".pcd");
  		transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

  		vfh_model histogram;
  		//ROS_INFO("NO load");
		loadHist (point, histogram);

		std::string kdtree_idx_file_name = "kdtree.idx";
		std::string training_data_h5_file_name = "training_data.h5";
		std::string training_data_list_file_name = "training_data.list";

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
		  int vote[2]={0,0};
		  //cube, star
		  for (int i = 0; i < k; ++i)
		  {
		  	pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
		        i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
		        //ROS_INFO("in1");	  
		  	if (k_distances[0][i]>150) continue;
		  	std::string name=models.at (k_indices[0][i]).first;
			std::size_t found = name.find("Cube");
			//ROS_INFO("in2");
  			if (found!=std::string::npos) vote[0]++;
  			found = name.find("Star");
  			//ROS_INFO("in3");	
  			if (found!=std::string::npos) vote[1]++;
  			//pcl::console::print_info ("end\n");
		  }
		  int order=-1;
		  int max=0;
		  for (int i=0;i<2;i++)
		  {
		  	if (vote[i]>max)
		  	{
		  		order=i;
		  		max=vote[i];
		  	}

		  }
		  if (order==-1) message="N/A";
		  if (order==0) message="Cube";
		  else if(order==1) message="Star";
		  // // delete[] k_indices.ptr();
		  // // delete[] k_distances.ptr();    
		  // // delete[] data.ptr();
		  //ROS_INFO("end of compare");
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




