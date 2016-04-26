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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Pose.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "opencv2/nonfree/nonfree.hpp"
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <tf_conversions/tf_kdl.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

class PtamScale
{
public:
	ros::NodeHandle nh;
	ros::Subscriber movewebcamrobot, stop_sub,ptam_sub;
	KDL::Frame frame_so3_ptam,frame_w_c;
	double scala;
	KDL::Frame So3_prev_ptam;
	std::ofstream myfile1,myfile,myfile4;
    bool stop_flag;
    std::vector<double> Robot;
	std::vector<double> Ptam;
	std::vector<double> Vect_scala;
	KDL::Frame Move_robot;
	ros::Publisher pub_scala;
	int count_n_passi;

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


#endif