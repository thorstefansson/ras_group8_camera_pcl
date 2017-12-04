#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h> //<pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>

//for shape vfh

#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>

//for cluster extraction:
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/lexical_cast.hpp>


//for publicing center as multiarray: ,... maybe not all needed
#include <stdio.h>
#include <stdlib.h>


#include "geometry_msgs/PointStamped.h"
#include <ras_group8_brain/Vision.h>

//to read and write file:
#include <fstream>
#include <string>

//for publishing arrays:
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
//for getting maximum value
#include<algorithm>


using namespace std;
using namespace pcl;

typedef std::pair<std::string, std::vector<float> > vfh_model;
typedef pcl::VFHSignature308 PointT;



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


class vision
{
public:
	ros::NodeHandle& nodeHandle_;
	ros::Publisher pub;

    //new:
    //ros::Publisher pub_color_probs;
    //ros::Publisher pub_shape_probs;
    std::vector<ros::Publisher> pub_color_probs, pub_shape_probs;
    //std::vector<ros::Publisher> pub_shape_probs;

	ros::Subscriber sub;
	int** result_matrix;
	int maxcluster;

	std::vector< vector<float> > shape_probs;
	std::vector< vector<float> > color_probs;
	ras_group8_brain::Vision memory;

    //new
    std::vector< vector<float> > color_probs2;
    std_msgs::Float32MultiArray color_probabilities, shape_probabilities;

	std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr> visioncloud_vec;

	int match[16][2]; 


	std::vector<vfh_model> models;
	flann::Matrix<int> k_indices;
	flann::Matrix<float> k_distances;
	flann::Matrix<float> data;

	vision(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
	{

		//ROS_INFO("start launch node 0002");
		maxcluster=1;
		initmatrix();

		
	  	

		for (int i =0;i<maxcluster;i++)
	  	{

	  		vector<float> new_shape;
	  		for (int j=0;j<8;j++) new_shape.push_back(0.0);
		  	shape_probs.push_back(new_shape);


		  	vector<float> new_color;
		  	for (int j=0;j<7;j++) new_color.push_back(0.0);
		  	color_probs.push_back(new_color);



            vector<float> new_color2;
            for (int j=0;j<9;j++) new_color2.push_back(0.0);
            color_probs2.push_back(new_color2);
	  	}
        memory.number[0]=0;
	  	memory.number[1]=0;
	  	memory.number[2]=0;


        //new:

         for (int i = 0; i < maxcluster; ++i)
         {
             std::string topicName2 = "/color_prob/cluster" + boost::lexical_cast<std::string>(i);
             ros::Publisher pub2 = nodeHandle_.advertise<std_msgs::Float32MultiArray> (topicName2, 1);
             pub_color_probs.push_back(pub2);

             std::string topicName3 = "/shape_prob/cluster" + boost::lexical_cast<std::string>(i);
             ros::Publisher pub3 = nodeHandle_.advertise<std_msgs::Float32MultiArray> (topicName3, 1);
             pub_shape_probs.push_back(pub3);
 }

		//initial record cloud


	  	//ROS_INFO("start color init");
		///inital color document
		  result_matrix = new int*[255];
		  ifstream resultFile;
          resultFile.open("/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera1/classifiers/colors_itr1e4_1e-5_gamma001_newer.dat");

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
		  	}
		  	resultFile.close();
		 //ROS_INFO("start shape init");

		  //init shape document
		std::string kdtree_idx_file_name = "/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/kdtree.idx";
		std::string training_data_h5_file_name = "/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/training_data.h5";
		std::string training_data_list_file_name = "/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/training_data.list";
		loadFileList (models, training_data_list_file_name);
		flann::load_from_file (data, training_data_h5_file_name, "training_data");
		pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
		   (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());





		sub = nodeHandle_.subscribe<sensor_msgs::PointCloud2> ("/visinput", 1, &vision::sensor_cb,this);

		pub = nodeHandle_.advertise<ras_group8_brain::Vision> ("/visoutput", 1);

        //pub2 = nodeHandle_.advertise<ras_group8_brain::Vision> ("/visoutput2", 1);

		ROS_INFO("successfully launch node 0223");




		  	///
	}

	void sensor_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
        ROS_INFO("cb");
		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
		pcl_conversions::toPCL(*cloud_msg, *cloud);
		//ROS_INFO("to process");
		sensor_msgs::PointCloud2  voxcloud=preprocess(cloud);
		//ROS_INFO("to seg");
		int num_clu=seg(voxcloud);
		//ROS_INFO("to recog");
        //std::cout<<"limit is "<<num_clu<<std::endl;

        //geometry_msgs::PointStamped msg;



		for (int i=0;i<maxcluster;i++)
		{
            std::cout<<"loop is "<<i<<std::endl;

			if (i>=num_clu)
			{
				memory.number[i]=0;
				//cout<<"!!!"<<endl;
				continue;
			}

            //new
            /*
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id="/camera_depth_optical_frame";
            msg.point.x=0;
            msg.point.y=0;
            msg.point.z=0;

            memory.position[i]=msg;
            ROS_INFO("go compute");
*/
			ROS_INFO("go color");
			color_recog(i);
			ROS_INFO("go shape");
			shape_recog(i);
			

			//showvect(i);


			ROS_INFO("go compute");
			compute(i);
			ROS_INFO("done compute");
            //alternative by thor:
            compute2(i);
		}




		//ROS_INFO("go publish");
		//cout<<memory.number[0]<<endl;
		pub.publish(memory);
		//OS_INFO("done publish");

	}


	sensor_msgs::PointCloud2 preprocess(const pcl::PCLPointCloud2* cloud)
	{
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		pcl::PCLPointCloud2* cloud_filtered= new pcl::PCLPointCloud2;
		pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (cloudPtr);
		sor.setMeanK(30);
		sor.setStddevMulThresh(2);
		sor.filter(*cloud_filtered);
		//return(cloud_filtered)

		pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud_filtered);
		pcl::PCLPointCloud2 cloud_filtered2;
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor2;
		sor2.setInputCloud (cloudPtr2);//(cloudPtr);
		  //Voxel Grid Filter
		sor2.setLeafSize (0.005, 0.005, 0.005);
		sor2.filter (cloud_filtered2);

		sensor_msgs::PointCloud2 output;//= new sensor_msgs::PointCloud2;
  		pcl_conversions::moveFromPCL(cloud_filtered2, output);
  		return output;
  	}

  	int seg(const sensor_msgs::PointCloud2& input)
  	{
  		pcl::PointCloud<pcl::PointXYZRGB> RGBcloud;
  		pcl::fromROSMsg (input, RGBcloud);


  		pcl::PointCloud<pcl::PointXYZHSV> cloud;
		pcl::PointCloud<pcl::PointXYZHSV> cloud_outliers;
		pcl::PointCloud<pcl::PointXYZHSV> cloud_inliers;

		PointCloudXYZRGBtoXYZHSV(RGBcloud, cloud); // RGB -> HSV


		// Instead of filtering out largest plane:
		/*
		pcl :: ModelCoefficients :: Ptr coefficients (new pcl :: ModelCoefficients  ());
		pcl :: PointIndices :: Ptr  inliers	(new pcl :: PointIndices  ());

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
		seg.segment (*inliers, *coefficients);
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
*/
		// Try to remove the floor given its coefficients:
  		//these are the coefficients of the floor: values: [-0.04023124650120735, -0.8639819622039795, -0.5019128918647766, 0.12330468744039536]

		pcl::SampleConsensusModelPlane<pcl::PointXYZHSV>::Ptr floor_model;
		pcl::IndicesPtr cloud_floor_inliers(new std::vector<int>);
		Eigen::VectorXf floor_coefficients(4);
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr HSVcloudPtr = cloud.makeShared();
		floor_model.reset(new pcl::SampleConsensusModelPlane<pcl::PointXYZHSV>(HSVcloudPtr));
		//this is only needed in initialization:
		float floor_coefficients_array [4] = {-0.04023124650120735, -0.8639819622039795, -0.5019128918647766, 0.12330468744039536};
		for (int i = 0; i<4 ; i++){
			floor_coefficients[i] =floor_coefficients_array[i];
		}

		floor_model->selectWithinDistance(floor_coefficients, 0.01, *cloud_floor_inliers);

        //cout << "floor inlier size" << cloud_floor_inliers.size() << endl;
  		// Create the filtering object
  		//pcl::ExtractIndices<pcl::PointXYZ> extract;
  		//WITH RGB:
		//  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  		//HSV:
		pcl::ExtractIndices<pcl::PointXYZHSV> extract;
  		//  Extract the inliers
		extract .setInputCloud (HSVcloudPtr);
		extract . setIndices ( cloud_floor_inliers );
  		// Extract outliers
  		extract.setNegative (true);				// Extract the outliers
  		extract.filter (cloud_outliers);		// cloud_outliers contains everything but the floor !!!!!!!
  		//ROS_INFO("%i",cloud_outliers.height*cloud_outliers.width);


		  // // Publish the model coefficients
		  // pcl::PointCloud<pcl::PointXYZHSV> downsampled_XYZ;


  		//   cloud_outliers = cloud_outliers;
  		  pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZHSV>);

		  tree->setInputCloud (cloud_outliers.makeShared());
		  //tree->setInputCloud (cloud_outliers);

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
		  ec.setMinClusterSize (60);

		  //Maximum size for colored objects to pick up around 220 points.
          //ec.setMaxClusterSize (400);  //25000 original. can use 500 for objects..

          ec.setMaxClusterSize (1500);

          ec.setSearchMethod (tree);
		  ec.setInputCloud (cloud_outliers.makeShared());

		  //ec.setInputCloud (cloud_outliers);
		  ec.extract (cluster_indices);

		  

		  visioncloud_vec.clear();

		  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		  {
		      pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZHSV>);

		      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		          cloud_cluster->points.push_back (cloud_outliers.points[*pit]); //*
		      cloud_cluster->width = cloud_cluster->points.size ();
		      cloud_cluster->height = 1;
		      cloud_cluster->is_dense = true;
		      visioncloud_vec.push_back(cloud_cluster);
		  }

		  return cluster_indices.size();


  	}

  	void color_recog(int order)
  	{
  		pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cluster (visioncloud_vec[order]);

		  double H;
		  double S;
		  double V;
		  double x=0;
	      double y=0;
	      double z=0;
	      float color_votes[9]={0.0};
	      pcl::PointXYZHSV point;

	      int orange_sat_limit = 0.7*255;
	      int orange_hue_limit = 90;

	      int green_sat_limit = 0.5*255;
	      int green_hue_limit = 210;
	      int number_of_color_votes = 0;

          int number_of_trap_votes = 0;


	      //in the dark already did: purple, red, orange,yellow, both blue, both green

	      int size = cloud_cluster->points.size () ;
          for (int i=0;i<size ; i++)
	      {
	        point = cloud_cluster->at(i);

	        x +=point.x;
	        y += point.y;
	        z += point.z;

	        H = point.h;
	        S = point.s;
	        S = S*254;
	        V = point.v;
			
            if(size<350){
			//For removing H and S values with high ambiguity:
	        if(S>orange_sat_limit || (S>green_sat_limit && H>orange_hue_limit) || H>green_hue_limit){
	        	number_of_color_votes++;
	        	color_votes[result_matrix[(int)S][(int)H]]++;
	        }
            else if (H>orange_hue_limit && H< green_hue_limit && S<green_sat_limit/2) {
                number_of_trap_votes++;
            }
            }
            else if(H>orange_hue_limit && H< green_hue_limit && S<green_sat_limit/1.5){
                number_of_trap_votes++;
            }

	      }


	      x = x/size;
	      y= y/size;
	      z=z/size;
	      for(int itr=0; itr<9;itr++) color_votes[itr] /= size;

	      geometry_msgs::PointStamped msg;

	      msg.header.stamp=ros::Time::now();
	      msg.header.frame_id="/camera_depth_optical_frame";
	      msg.point.x=x;
	      msg.point.y=y;
	      msg.point.z=z;

	      memory.position[order]=msg;



          // std::cout << "orange:" << color_votes[0] << " red:" << color_votes[1] << " yellow:" << color_votes[2]
          //              << " purple:" << color_votes[3] << " blue:" << color_votes[4] << " dark_green:" << color_votes[5]
          //                 <<" light green:" << color_votes[6] <<" battery:" << color_votes[7] << " trap:" << color_votes[8]
          //                   << endl;

	      //vector<float> p=(color_probs[order]);
          color_probabilities.data.clear();

          for (int itr1 = 0; itr1<9; itr1 ++) color_probabilities.data.push_back(color_votes[itr1]);
          pub_color_probs[order].publish(color_probabilities);

			for (int j=0;j<9;j++)
			{
				//std::cout<<i<<std::endl;
				if (j<=5) (color_probs[order])[j]=color_votes[j];
				else if (j==6) (color_probs[order])[5]+=color_votes[j];
				else if (j==7) (color_probs[order])[6]=color_votes[j];
				else if (j==8) (color_probs[order])[6]+=color_votes[j];
			}
\
            //new
            for (int j=0;j<8;j++)
            {
                (color_probs2[order])[j]=color_votes[j];
            }

            (color_probs2[order])[7] = 0;
			//if we do not get many color votes:
            if((float)number_of_color_votes/size < 0.2){
				//label the cluster as obstacle:
                cout << "number of color votes: " << number_of_color_votes << endl;
                cout << "number of trap votes: " << number_of_trap_votes << endl;
                cout << "size: " << size << endl;
				(color_probs[order])[6] = 1;

                if(number_of_trap_votes/size > 0.4) (color_probs2[order])[8] = 1;
                else (color_probs2[order])[7] = 1;
			}

	 }
	void shape_recog(int order)
	{
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cluster (visioncloud_vec[order]);
		pcl::NormalEstimation<pcl::PointXYZHSV, pcl::Normal> ne;
		ne.setInputCloud (cloud_cluster);
		pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZHSV> ());
		ne.setSearchMethod (tree);

		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
		ne.setRadiusSearch (0.03);

	  	// Compute the features
	  	ne.compute (*normals);

		pcl::VFHEstimation<pcl::PointXYZHSV, pcl::Normal, pcl::VFHSignature308> vfh;
		vfh.setInputCloud (cloud_cluster);
		vfh.setInputNormals (normals);
		vfh.setSearchMethod (tree);
		  // Output datasets
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
		//pcl::PointCloud<pcl::VFHSignature308> vfhs;
		  // Compute the features
		vfh.compute (*vfhs);
		//ROS_INFO("vfhs loaded,  compare, no pointer");

		compare(*vfhs,order);

	}




	////help functions

		bool
	loadHist (const pcl::PointCloud<pcl::VFHSignature308> &point, vfh_model &vfh)
	{
	  int vfh_idx;

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

	void compare(const pcl::PointCloud<pcl::VFHSignature308> &point, int inmum)
	{
		int k =10;
		int shapesize=8;

  		double thresh = 50;     // No threshold, disabled by default
  		

  		vfh_model histogram;
  		//ROS_INFO("NO load");
		loadHist (point, histogram);

		

		
		  // Check if the data has already been saved to disk
		    
		  

		  // Check if the tree index has already been saved to disk
		    //ROS_INFO("No flann");
		 
		    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("/home/ras18/catkin_ws/src/ras_group8/ras_group8_camera_pcl/kdtree.idx"));
		    index.buildIndex ();
		    nearestKSearch (index, histogram, k, k_indices, k_distances);

            //ROS_INFO("complete knn");
		  float vote[8]={0,0,0,0,0,0,0,0};
		  //for (int t=0;t<8;t++) vote[t]=0;
		  //cube, holcube,sphere,holtriangle,holcylinder,holcross,star
		  for (int i = 0; i < k; ++i)
		  {
		  	//pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
		        //i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
		        //ROS_INFO("in1");	  
		  	std::string name=models.at (k_indices[0][i]).first;
			std::size_t found = name.find("Cube");
			//ROS_INFO("");
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

  			found = name.find("Trap");
  			if (found!=std::string::npos) vote[7]+=1/(k_distances[0][i]);

  			//pcl::console::print_info ("end\n");
		  }

		  //normalize
		  double dsum=0;
		  for (int i=0;i<shapesize;i++) dsum+=vote[i];
		  for (int i=0;i<shapesize;i++) vote[i]/=dsum;



          //new:
          shape_probabilities.data.clear();
          for (int itr1 = 0; itr1<9; itr1 ++) shape_probabilities.data.push_back(vote[itr1]);
          pub_shape_probs[inmum].publish(shape_probabilities);



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


		  //vector<float> p=shape_probs[inmum];

			for (int j=0;j<8;j++)
			{
				//cout<<"error at "<<j<<" where value "<<vote[j]<<endl;
				
				(shape_probs[inmum])[j]=vote[j];
			}
		  
		
      	//pub_shape_probabilities.publish(shape_probabilities);
	}


	void initmatrix()
	{
		
		int temp[16][2]={
			{0,5},{0,4},{0,2},
			{1,1},{1,5},
			{2,1},{2,2},
			{3,4},
			{4,1},{4,5},
			{5,3},{5,0},
			{6,0},{6,3},
			{0,6},{7,6}

		};
		for (int i=0;i<16;i++) 
			{
				match[i][0]=temp[i][0];
				match[i][1]=temp[i][1];
			}


		
	}

	void compute(int order)
	{


		//ROS_INFO("compute");
		vector<float> ps=shape_probs[order];
		vector<float> pc=color_probs[order];
		// std::cout<<ps[0]<<ps[1]<<std::endl;
		// std::cout<<pc[0]<<pc[1]<<std::endl;
		//ROS_INFO("compute0");
		float combine_prob[17]={0};
		//ROS_INFO("compute 1");
		for (int i=1;i<17;i++)
		{
			int x=match[i-1][0];
			int y=match[i-1][1];
			combine_prob[i]=ps[x]*pc[y];
			if (combine_prob[i]>1) combine_prob[i]=0;
		}
		//ROS_INFO("compute2");
		float sum=0;
		for (int i=1;i<17;i++) sum+=combine_prob[i];
		for (int i=1;i<17;i++) combine_prob[i]/=sum;

		int index=0;
		float max=0.3;
		for (int i=1;i<17;i++)
		{
			if (combine_prob[i]>max)
			{
				index=i;
				max=combine_prob[i];
			}
		}
		//ROS_INFO("compute3");
        //memory.number[order]=index;
        //std::cout<<" the result index is "<<index<<std::endl;


	}


    void compute2(int order)
    {


        //ROS_INFO("compute");
        vector<float> ps=shape_probs[order];
        vector<float> pc=color_probs2[order];
        //Available objects are:
        /*
1    Red Cube
2    Red Hollow Cube
3    Red Ball
4    Red hollow cylinder

5    (dark)Green Cube
6    (light)Green Hollow Cube
7    (light)Green hollow Cylinder

8    Yellow Cube
9    Yellow Ball

10    Blue Cube
11   Blue Hollow Triangle

12    Purple Cross
13    Purple Star

14    Orange cross
15    Patric (the orange star)

16 obstacle..
17 trap
*/
        //order of colors is
        /*
         *  "orange:" 0  red: "1" yellow:"  2<< " purple:" 3<< " blue:"  <<   "4 dark_green:"   5 < light green:"6 <<  obstacle: 7 << trap 8<<
          */
        //order of shapes is:
        /*
   0"Cube")
1"Holcu");
2("Sphere");
3"Holtri");
4"Holcy");
5"Holcro");
6("Star");
7("Trap")*/
/*
        cout<<"color vector 2 is ";
        for (int i=0;i<8;i++)
        {
            cout<<" "<<pc[i];
        }
        cout<<endl;
*/
        string object [18]= {"Red Cube", "Red Hollow Cube", "Red Ball", "Red Hollow Cylinder", "Green Cube", "Green Hollow Cube",
                             "Green Hollow Cylinder", "Yellow Cube", "Yellow Ball", "Blue Cube", "Blue Hollow Triangle",
                             "Purple Cross", "Purple Star", "Orange Cross", "Patric","obstacle", "trap"};
        //indexes:
        int red_indices[4] = {0,1,2,4};
        //int dark_green_indices only 0
        /*
        int light_green_indices = {1,4};
        int yellow_indices = {0, 2};
        int blue_indices = {0, 3};
        int purple_indices = {5, 6};
        int orange_indices = {5, 6};
*/
        float current_max;
        int current_max_index;
        int object_number;


        //cout << "index is: " << distance(color_probs2.begin(), max_element (color_probs2.begin(),color_probs2.end())) << endl;

        switch(distance(pc.begin(), max_element (pc.begin(),pc.end()))){

        case 0: {
            cout << "case 0 "<< endl;
            if(ps.at(5)>ps.at(6)) object_number = 14;
            else object_number = 15;
            break;
        }
        case 1: {
            current_max = ps.at(0);
            current_max_index = red_indices[0];
            for(int i=1;i<4;i++){
                if(ps.at(red_indices[i])>current_max){
                    current_max_index = i;
                    current_max = pc.at(red_indices[i]);}
            }
            object_number =  current_max_index +1;
            break;
        }

        case 2: {
            if(ps.at(0)>ps.at(2)) object_number = 8;
            else object_number = 9;
            break;
        }

        case 3: {
            if(ps.at(5)>ps.at(6)) object_number = 12;
            else object_number = 13;
            break;
        }
        case 4: {
            if(ps.at(0)>ps.at(3)) object_number = 10;
            else object_number = 11;
            break;
        }
        case 5: {
            object_number = 5;
            break;
        }
        case 6: {
            if(ps.at(1)>ps.at(4)) object_number = 6;
            else object_number = 7;
            break;
        }
        case 7: {
            //cout << "case 7 "<< endl;
            object_number = 16;
            break;
        }
        case 8: {
            object_number = 17;
            break;
        }
        }

        cout << "The object is perhaps: " << object[object_number-1] << endl;

        //cout << "The object number is " << object_number << endl;
        memory.number[order]=object_number;

    }

	void showvect(int order)
	{
		vector<float> ps=shape_probs[order];
		vector<float> pc=color_probs[order];

        /*
		cout<<"color vector is ";
		for (int i=0;i<7;i++)
		{
			cout<<" "<<pc[i];
		}
		cout<<endl;


		cout<<"shape vector is ";
		for (int i=0;i<8;i++)
		{
			cout<<" "<<ps[i];
		}
        cout<<endl;*/
	}

};




int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "vision");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(0.5);
  vision vs(nodeHandle);

  for (;;) {
      ros::spinOnce();
      loop_rate.sleep();

  }
  ros::spin();
  return 0;
}
