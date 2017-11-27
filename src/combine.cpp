#include <ros/ros.h>

#include <iostream>
#include "string"
#include <std_msgs/String.h>
#include "std_msgs/Float32MultiArray.h"


class combine
{
public:	
	ros::NodeHandle& nodeHandle_;

  	std::vector<ros::subscriber> sub_shape_vec;

  	std::vector<ros::Subscriber> sub_color_vec;


	int maxcluster;
	std_msgs::Float32MultiArray shape_probabilities;
	std::vector<float32*> shape_probs;
	std::vector<float32*> color_probs;


	int match[7][9]; 

	// float32[] shape0;
	// float32[] shape1;
	// float32[] shape2;
	// float32[] color0;
	// float32[] color1;
	// float32[] color2;

	combine(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle)
	{
		// if (!readParameters()) {
	 //    ROS_ERROR("Could not read parameters.");
	 //    ros::requestShutdown();
	 //  

	  maxcluster=3;
	  for (int i =0;i<maxcluster;i++)
	  {

	  	std::string shapeName = "/Object_detection/shape_prob/cluster" + boost::lexical_cast<std::string>(i);
	  	std::string colorName = "/Object_detection/color_prob/cluster" + boost::lexical_cast<std::string>(i);
	  	ros::Subscriber sub_s;
	  	ros::Subscriber sub_c;


	  	if (i==0) 
	  		{
	  			sub_s=nodeHandle_.subscribe(shapeName,1,&combine::shapeCB0,this);
	  			sub_c=nodeHandle_.subscribe(colorName,1,&combine::colorCB0,this);
	  		}
	  	if (i==1) 
	  		{
	  			sub_s=nodeHandle_.subscribe(shapeName,1,&combine::shapeCB1,this);
	  			sub_c=nodeHandle_.subscribe(colorName,1,&combine::colorCB1,this);
	  		}
	  	if (i==2) 
	  		{
	  			sub_s=nodeHandle_.subscribe(shapeName,1,&combine::shapeCB2,this);
	  			sub_c=nodeHandle_.subscribe(colorName,1,&combine::colorCB2,this);
	  		}

	  	sub_shape_vec.push_back(sub_s);
	  	sub_color_vec.push_back(sub_c);

	  	float32* new_shape=new float[7]{0,0,0,0,0,0,0};
	  	shape_probs.push_back(new_shape);
	  	float32* new_color=new float[9]{0,0,0,0,0,0,0,0,0};
	  	shape_probs.push_back(new_shape);

	  }

	  ros::Publisher pub = nodeHandle_.advertise<std_msgs::Float32MultiArray> ("/Object_detection/object", 1);
	  
	  // shape_pub_ =
   //  	nodeHandle_.advertise<std_msgs::String>("/Object_detection/shape", 1);

   //  	pub_shape_probabilities = nodeHandle_.advertise<std_msgs::Float32MultiArray>("/object_classification/shape_probabilities", 1);
	  ROS_INFO("Successfully launchednode different cb.");



	}
	virtual ~combine()
	{
	}

	
	// bool readParameters()
	// {
 //    	if (!nodeHandle_.getParam("modname",
 //                            modname))
 //    		return false;

 //    	return true;


	// }

	///help functions

	void shapehelp(int order, float32* indata )
	{
		int maxsize=shape_probs[order].size();
		for (int i=0;i<maxsize;i++)
		{
			float32* p=shape_probs[order];
			p[i]=indata[i];
		}

	}

	void colorhelp(int order,float32* indata)
	{
		int maxsize=color_probs[order].size();
		for (int i=0;i<maxsize;i++)
		{
			float32* p=color_probs[order];
			p[i]=indata[i];
		}

	}

	////call back functions

	void shapeCB0(const std_msgs::Float32MultiArrayPtr& input)
	{
		float* indata=input->data;
		shapehelp(0,indata);
			//pcl::PCDWriter writer;
			//ROS_INFO("end of cb0 ");
	}

	void colorCB0(const std_msgs::Float32MultiArrayPtr& input)
	{
			//pcl::PCDWriter writer;
		float* indata=input->data;
		colorhelp(0,indata);
					//ROS_INFO("end of cb1 ");
	}

	void shapeCB1(const std_msgs::Float32MultiArrayPtr& input)
	{
		float* indata=input->data;
		shapehelp(1,indata);
			//pcl::PCDWriter writer;
			//ROS_INFO("end of cb0 ");
	}

	void colorCB1(const std_msgs::Float32MultiArrayPtr& input)
	{
			//pcl::PCDWriter writer;
		float* indata=input->data;
		colorhelp(1,indata);
					//ROS_INFO("end of cb1 ");
	}

	void shapeCB2(const std_msgs::Float32MultiArrayPtr& input)
	{
		float* indata=input->data;
		shapehelp(2,indata);
			//pcl::PCDWriter writer;
			//ROS_INFO("end of cb0 ");
	}

	void colorCB2(const std_msgs::Float32MultiArrayPtr& input)
	{
			//pcl::PCDWriter writer;
		float* indata=input->data;
		colorhelp(2,indata);
					//ROS_INFO("end of cb1 ");
	}

	////combine all the at and output

	void compute()
	{

	}
	


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_combine");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(1);
  combine rg(nodeHandle);

  for (;;) {
      ros::spinOnce();
      loop_rate.sleep();

  }
  ros::spin();
  return 0;
}




