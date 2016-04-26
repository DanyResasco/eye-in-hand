#ifndef Pollini_project_h
#define Pollini_project_h
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

// using namespace cv;
/*!
	*test
*/


class Camera
{
	public:
		ros::NodeHandle nh;
    	cv::Mat Camera_Matrix;
    	cv::Mat Cam_par_distortion;
    	cv::Mat Camera2_S03;  
    	KDL::Frame Move_robot;  
		cv::Mat scene;
		int arrived_cam = 0;		
		ros::Subscriber movewebcamrobot, scala_sub;
		ros::Subscriber ptam_kf3d;
		image_transport::ImageTransport it_;
	  	image_transport::Subscriber sub;
	  	ros::Subscriber ptam_sub;
	
		struct Obj 
		{
			std::vector<cv::Point> Bot_C; //contour
			cv::Point Center_;
			std::vector<cv::KeyPoint> keyp_;
			cv::Mat descr_;
			cv::Mat figure_;
			cv::Point3d Pos3d_;
			cv::Point Botton_2frame;
		} BottonCHosen;

		
		std::vector<cv::Point2f> KeypointIm2;
		std::vector<cv::Point2f> KeyPointIm1Match;
		
		KDL::Frame frame_so3_ptam;

	  	std::string camera_topic_;
	  	static cv::Point pos_object;
		static int press_buttom;
		static int first_Step ;
		
		bool move_camera_end;		
		int FirstCalibration;
		cv::Mat frame1_;	
		double scala;
		KDL::Frame So3_prev_ptam;
		bool SaveFirst;
		bool sub_ptam_2;
				
		float cam_fx, cam_d0, cam_d1, cam_d2, cam_d3,cam_d4, cam_fy, cam_cx, cam_cy;

		pcl::PointCloud<pcl::PointXYZ> Ptamkf3d;
		KDL::Frame frame_w_c;	///camere in word
		
		int count_n_passi;
		std::vector<double> Robot;
		std::vector<double> Ptam;
		std::ofstream myfile1,myfile,myfile4;
    	bool stop_flag;
    	bool InfoKf3d_;



		/*!
			\brief constructor
		*/
		Camera();

		/*!
			\brief destructor
		*/
		~Camera(){};

		/*!
			\brief Function that handle the first iteration.
		*/
		void ControllCamera();

		/*!
			\brief Function that calculates the keypoint for the second image, and match it with the first image
		*/
		void DetectWithSift();
		
		/*!
			\brief Function that triangulate the point
		*/
		void Triangulation();
		
	

	private:
		
		/*!
			\brief Function that save the 2d position of press botton 
		*/
		static void CallBackFunc(int event, int x, int y, int flags, void* userdata);

		/*!
			\brief Function that find the botton desired
		*/
		void ShapeDetect();
		std::pair<int, bool> FindAMinDistanceButton(std::vector<cv::Point> &baricentro, cv::Point &point_);
		
		/*!
			\brief Callback that convert sensor image with cv mat
		*/
		void ImageConverter(const sensor_msgs::Image::ConstPtr& msg);

		/*!
			\brief Approximate contour with accuracy proportional to the contour perimeter
			\return Center of each shape and his contours
		*/
		std::pair<std::vector<cv::Point> ,std::vector<std::vector<cv::Point>> > FindContours(cv::Mat bw, cv::Mat camera);
		
		/*!
			\brief Pose word into camera
		*/
		void SOtreCamera(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
		
		/*!
			\brief Callback that save the real movement of robot

			This pose is relative to frame early
		*/	
		void RobotMove(const geometry_msgs::Pose msg);

		/*!
			\brief Callback that convert PointCloud2 to poincloudXYX

			This pose is relative to word frame.
		*/
		void InfoKf3d(const sensor_msgs::PointCloud2::ConstPtr& msg);

		/*!
			\brief Reproject the points 3d to 2d and find the 3d botton pose
			\param: pose of each feature

			To find the 3d botton pose, we fit a plane with lms method and project the 2d botton position into this plane
		*/
		void ProjectPointAndFindPosBot3d(std::vector<cv::Point3d> vect3d);

		void FillCamMatrixPose(KDL::Frame frame);

		/*!
			\brief Convert each point in camera frame
			\return 3d point in camera frame

			Scaling each point with the scale factor.
		*/
		std::vector<cv::Point3d> ConvertPointFromWordToCam();
		
		/*!
			\brief Find the 3d botton pose
			\param [in]: plane param

			Project the 2d botton position into 3d plane
		*/
		void FindBottonPos3D(Eigen::Vector4f plane_param);
		// void StopCallback(const std_msgs::Bool::ConstPtr& msg);
		void ScaleCallback(const std_msgs::Float32::ConstPtr msg);
};


/*!
	\brief Plane estimation
	\param [in] 3d position of each point near the botton

	Estimate the plane with lms method
*/
Eigen::Vector4f EstimatePlane(std::vector<cv::Point3d> Point_Near);

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

/*! \brief Find a centroid of shape 
	\param [in] shape contours
	\return center position
*/
cv::Point FindACenter(std::vector<cv::Point> &geometry);


/*! \brief function that calculates the max value of width and height for the roi  
	\param [in] mat of scene
	\param [in] point of interest
	\return range to cut the image to make a roi 
*/
std::pair<int,int> FindMaxValue(cv::Mat &matrix, cv::Point &point );



cv::Point Camera::pos_object;
int Camera::press_buttom = 0;
int Camera::first_Step = 1;


double Media(cv::Mat triangulatedPoints3D, double MaxLenght, int col);

/*! \brief Convert cv point into eigen vector
	\param [in] cv::Point3d
	\param [out]  Eigen::Vectorxd homogeneus
*/
void FromCvPointToEigen(cv::Point3d point_, Eigen::VectorXd &vect);

/*! \brief Convert cv mat into eigen matrix
	\param [in] cv::Mat
	\param [out]  Eigen::MatrixXd homogeneus
*/
void FromMatToEigen(cv::Mat Mat_, Eigen::MatrixXd &Eigen);

/*! \brief Convert eigen vector to cv point
	\param [in] Eigen::VectorXd
	\param [out] cv::Point3d
*/
void FromEigenVectorToCvPOint(Eigen::VectorXd Eigen, cv::Point3d &mat);

/*! \brief Calculates the scale factor
	\param [in] actual position
	\param [in] previusly position
	\param [in] robot movements
	\return scale factor
*/
double ScalaReturn(double ptam, double ptam_prev, double robot);





double Scale(std::vector<double> X, std::vector<double> Y) ;

double standard_deviation(std::vector<double> data);
double Scale_factor(std::vector<double> X, std::vector<double> Y) ;



#endif