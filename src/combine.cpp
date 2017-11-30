#include <ros/ros.h>

#include <iostream>
#include "string"
#include <std_msgs/String.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PointStamped.h"
#include <ras_group8_brain/Vision.h>


class combine
{
public:	
	ros::NodeHandle& nodeHandle_;

  	std::vector<ros::Subscriber> sub_shape_vec;

  	std::vector<ros::Subscriber> sub_color_vec;

  	std::vector<ros::Subscriber> sub_pos_vec;

  	ros::Publisher pub;




	int maxcluster;
	std_msgs::Float32MultiArray shape_probabilities;
	std::vector<float*> shape_probs;
	std::vector<float*> color_probs;
	ras_group8_brain::Vision memory;


	int colornum;
	int shapenum;


	 int match[16][2]; 
	 

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
		colornum=7;
		shapenum=8;


		///initial matrix


		initmatrix();

	  maxcluster=3;
	  for (int i =0;i<maxcluster;i++)
	  {

	  	std::string shapeName = "/Object_detection/shape_prob/cluster" + boost::lexical_cast<std::string>(i);
	  	std::string colorName = "/Object_detection/color_prob/cluster" + boost::lexical_cast<std::string>(i);
	  	std::string posName = "/Object_detection/pos/cluster" + boost::lexical_cast<std::string>(i);
	  	ros::Subscriber sub_s;
	  	ros::Subscriber sub_c;
	  	ros::Subscriber sub_p;


	  	if (i==0) 
	  		{
	  			sub_s=nodeHandle_.subscribe(shapeName,1,&combine::shapeCB0,this);
	  			sub_c=nodeHandle_.subscribe(colorName,1,&combine::colorCB0,this);
	  			sub_p=nodeHandle_.subscribe(posName,1,&combine::posCB0,this);
	  		}
	  	if (i==1) 
	  		{
	  			sub_s=nodeHandle_.subscribe(shapeName,1,&combine::shapeCB1,this);
	  			sub_c=nodeHandle_.subscribe(colorName,1,&combine::colorCB1,this);
	  			sub_p=nodeHandle_.subscribe(posName,1,&combine::posCB1,this);
	  		}
	  	if (i==2) 
	  		{
	  			sub_s=nodeHandle_.subscribe(shapeName,1,&combine::shapeCB2,this);
	  			sub_c=nodeHandle_.subscribe(colorName,1,&combine::colorCB2,this);
	  			sub_p=nodeHandle_.subscribe(posName,1,&combine::posCB2,this);
	  		}

	  	sub_shape_vec.push_back(sub_s);
	  	sub_color_vec.push_back(sub_c);
	  	sub_pos_vec.push_back(sub_p);

	  	float new_shape[8]={0,0,0,0,0,0,0,0};
	  	shape_probs.push_back(new_shape);
	  	float new_color[7]={0,0,0,0,0,0,0};
	  	color_probs.push_back(new_shape);
	  	

	  }
	  memory.number[0]=0;
	  memory.number[1]=0;
	  memory.number[2]=0;

	  pub = nodeHandle_.advertise<ras_group8_brain::Vision> ("/Object_detection/object", 1);
	  
	  // shape_pub_ =
   //  	nodeHandle_.advertise<std_msgs::String>("/Object_detection/shape", 1);

   //  	pub_shape_probabilities = nodeHandle_.advertise<std_msgs::Float32MultiArray>("/object_classification/shape_probabilities", 1);
	  ROS_INFO("Successfully launchednode 1118");



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


	void shapehelp(int order, float* indata )
	{
		ROS_INFO("shapehelp");
		int maxsize=8;
		for (int i=0;i<maxsize;i++)
		{
			float* p=shape_probs[order];
			p[i]=indata[i];
		}
		compute(order);

	}

	void colorhelp(int order,float* indata)
	{
		ROS_INFO("colorhelp");
		//int vecsize=indata.size();
		int maxsize=9;
		
		for (int i=0;i<maxsize;i++)
		{
			//std::cout<<i<<std::endl;
			float* p=color_probs[order];
			if (i<=5) p[i]=indata[i];
			else if (i==6) p[5]+=indata[i];
			else if (i==7) p[6]=indata[i];
			else if (i==8) p[6]+=indata[i];
		}
		compute(order);

	}

	////call back functions

	void posCB0(const geometry_msgs::PointStampedPtr& input)
	{
		ROS_INFO("pos0");
		memory.position[0]=*input;
	}


	void shapeCB0(const std_msgs::Float32MultiArrayPtr& input)
	{
		float indata[8]={0};
		for (int i=0;i<8;i++)
		{
			indata[i]=input->data[i];
		}
		shapehelp(0,indata);
			//pcl::PCDWriter writer;
			//ROS_INFO("end of cb0 ");
	}

	void colorCB0(const std_msgs::Float32MultiArrayPtr& input)
	{
			//pcl::PCDWriter writer;
		float indata[9]={0};
		for (int i=0;i<9;i++)
		{
			indata[i]=input->data[i];
		}
		colorhelp(0,indata);
					//ROS_INFO("end of cb1 ");
	}

	void posCB1(const geometry_msgs::PointStampedPtr& input)
	{
		ROS_INFO("pos1");
		memory.position[1]=*input;
			//pcl::PCDWriter writer;
			//ROS_INFO("end of cb0 ");
	}

	void shapeCB1(const std_msgs::Float32MultiArrayPtr& input)
	{
		float indata[8]={0};
		for (int i=0;i<8;i++)
		{
			indata[i]=input->data[i];
		}
		shapehelp(1,indata);
			//pcl::PCDWriter writer;
			//ROS_INFO("end of cb0 ");
	}

	void colorCB1(const std_msgs::Float32MultiArrayPtr& input)
	{
			//pcl::PCDWriter writer;
		float indata[9]={0};
		for (int i=0;i<9;i++)
		{
			indata[i]=input->data[i];
		}
		colorhelp(1,indata);
					//ROS_INFO("end of cb1 ");
	}

	void posCB2(const geometry_msgs::PointStampedPtr& input)
	{
		ROS_INFO("pos2");
		memory.position[2]=*input;
			//pcl::PCDWriter writer;
			//ROS_INFO("end of cb0 ");
	}

	void shapeCB2(const std_msgs::Float32MultiArrayPtr& input)
	{
		float indata[8]={0};
		for (int i=0;i<8;i++)
		{
			indata[i]=input->data[i];
		}
		shapehelp(2,indata);
			//pcl::PCDWriter writer;
			//ROS_INFO("end of cb0 ");
	}

	void colorCB2(const std_msgs::Float32MultiArrayPtr& input)
	{
			//pcl::PCDWriter writer;
		float indata[9]={0};
		for (int i=0;i<9;i++)
		{
			indata[i]=input->data[i];
		}
		colorhelp(2,indata);
					//ROS_INFO("end of cb1 ");
	}

	////combine all the at and output

	void compute(int order)
	{
		//ROS_INFO("compute");
		float* ps=shape_probs[order];
		float* pc=color_probs[order];
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

		memory.number[order]=index;
		if (index!=0) publishtopic();



	}

	void publishtopic()
	{
		
		pub.publish(memory);
		ROS_INFO("pub");

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




