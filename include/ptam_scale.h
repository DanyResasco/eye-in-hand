#ifndef Pollini_p_h
#define Pollini_p_h
#include <utility>
#include <list>
#include <string>

#include <eigen3/Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames_io.hpp>
#include <ptam_com/KeyFrame_msg.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"


#include "opencv2/nonfree/nonfree.hpp"
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <tf_conversions/tf_kdl.h>


#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

class PtamScale
{
public:
	ros::NodeHandle nh;
	ros::Subscriber movewebcamrobot, stop_sub,ptam_sub;
	KDL::Frame frame_so3_ptam,frame_w_c;
	KDL::Frame So3_prev_ptam;
	std::ofstream myfile4;
    bool stop_flag;
	KDL::Frame Move_robot;
	ros::Publisher pub_scala, pub_scala_naif;
	std::vector<double> Robot;
	std::vector<double> Ptam;
	std::vector<double> Vect_scala;

		PtamScale();
	~PtamScale(){};

private:
		


		/*!
			\brief Pose word into camera
		*/
		void SOtreCamera(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
		
		/*!
			\brief Callback that save the real movement of robot

			This pose is relative to frame early
		*/	
		void RobotMove(const geometry_msgs::Pose msg);

		void StopCallback(const std_msgs::Bool::ConstPtr& msg);
};


double Scale(std::vector<double> X, std::vector<double> Y) ;
double standard_deviation(std::vector<double> data);
double ScalaReturn(double ptam, double ptam_prev, double robot);

#endif