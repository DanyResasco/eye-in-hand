#ifndef Pollini_project_h
#define Pollini_project_h


#include <iostream>
#include <utility>
#include <list>
#include <string>
#include <stdio.h>
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

//sift
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
 #include <tf/transform_broadcaster.h>

// #include "opencv2/imgcodecs.hpp"

// #include "opencv2/core/utility.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <tf_conversions/tf_kdl.h>
#include <std_srvs/Empty.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

using namespace cv;



class Camera
{
	public:
		ros::NodeHandle nh;
		// std::vector<KDL::Frame> frame_vect, frame_vect2d;

		std::vector<cv::Point3d> frame_vect;
		std::vector<cv::Point2d> frame_vect2d;

		// double video_or_photo;
    	image_transport::Publisher image_pub_;
    	ros::Subscriber ptam_sub;
    	cv::Mat Camera_Matrix;
    	cv::Mat Cam_par_distortion;
    	cv::Mat Camera2_S03;
    	cv::Mat Im1;
    	cv::Mat Im2;
    	KDL::Frame Move_robot;
    	KDL::Rotation Move_robot_prev;
    	// cv::Mat Ptamkf3d;

		cv::Mat scene;
		int arrived_cam = 0;
		double ptam_scale;
		bool sub_ptam;
		cv::Mat mat_;
		ros::Subscriber srv_move,movewebcamrobot;
	
		struct Obj 
		{
			std::vector<cv::Point> Bot_C; //contour
			cv::Point Center_;
			std::vector<KeyPoint> keyp_;
			cv::Mat descr_;
			cv::Mat figure_;
			cv::Point3d Pos3d_;
		} BottonCHosen;

		cv::Mat Point_3d; 
		cv::Point Botton_2frame;
		double move_z_robot;

		std::vector<cv::Point2f> KeypointIm2;
		std::vector<Point2f> KeyPointIm1Match;
		cv::Mat triangulatedPoints3D;
		KDL::Frame frame_so3_ptam;

		image_transport::ImageTransport it_;
	  	image_transport::Subscriber sub;
	  	std::string camera_topic_;
	  	static Point pos_object;
		static int press_buttom;
		static int first_Step ;
		double Finish = 1;
		bool move_camera_end;
		bool read_ptam;
		int start = 0;
		cv::Mat Image_roi2;
		int FirstCalibration;
		cv::Mat frame1_;
		double RobotArmLenght;
		double media_z, media_x,media_y ;
		KDL::Vector scala;
		double distanzaWebcam;
		KDL::Frame So3_prev_ptam;
		// KDL::Vector So3_prev_ptam;
		KDL::Frame S03_prev_robot;
		bool SaveFirst;
		bool sub_ptam_2;
		// double scala;
		cv::Mat Point3dTriangulate;
		bool scala_ok;
		ros::Subscriber ptam_kf3d;
		cv::Mat scene_first;
		std::vector<cv::Point2d> projectedPoints;
		// std::vector<cv::Point3d> vect3d;
		float cam_fx, cam_d0, cam_d1, cam_d2, cam_d3,cam_d4, cam_fy, cam_cx, cam_cy;
		std::vector<cv::Point2d> kf_point;
		std::vector<cv::Point3d> kf_point3d;
		ros::Subscriber dany_kf,dany_kf3d,ptam_kf2d;

			std::vector<cv::Point3d> point_near;
				std::vector<int> index_near;
				bool sub_ptam1;
				ros::Publisher marker_pub;
		pcl::PointCloud<pcl::PointXYZ> Ptamkf3d;
		KDL::Frame frame_w_c;	//camere in word
		bool scala_first;


		Camera();
		~Camera(){};
		void ControllCamera();
		void DetectWithSift();
		void StereoCalibration();
		void Triangulation();
		void MatchPtamCv(std::vector<cv::Point2d> vect);
		void Find3dPoint();

	private:
		
		static void CallBackFunc(int event, int x, int y, int flags, void* userdata);
		void ShapeDetect();
		std::pair<int, bool> FindAMinDistanceButton(std::vector<cv::Point> &baricentro, cv::Point &point_);
		void ImageConverter(const sensor_msgs::Image::ConstPtr& msg);
		std::pair<std::vector<cv::Point> ,std::vector<std::vector<cv::Point>> > FindContours(cv::Mat bw, cv::Mat camera);
		void SOtreCamera(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
		
		void CreateAVector(std::vector<Point2f> keyp2, std::vector<Point2f> keyp_1 , cv::Mat &key_array_1, cv::Mat &key_array_2);
		void MoveCallBack(const std_msgs::Bool::ConstPtr msg);
		void FindXeY(cv::Mat cameraMatrix, double media_z, cv::Mat tvec);
		void RobotMove(const geometry_msgs::Pose msg);
		void InfoKf3d(const sensor_msgs::PointCloud2::ConstPtr& msg);
		// void InfoKf3d(const ptam_com::KeyFrame_msg::ConstPtr msg);
		void InfoKf2d(const std_msgs::Float32MultiArray::ConstPtr &msg);
		// void InfoKf3d(const std_msgs::Float32MultiArray::ConstPtr &msg);
		// std::vector<cv::Point3d> ProjectPointWithPtamCamParam(std::vector<cv::Point3d> vect3d);
		void POSE3d(const std_msgs::Float32MultiArray::ConstPtr msg);
		void POSE2d(const std_msgs::Float32MultiArray::ConstPtr msg);
		void ProjectPointAndFindPosBot3d(std::vector<cv::Point3d> vect3d);

		void FillCamMatrixPose(KDL::Frame frame);
		void FindScale();

		std::vector<cv::Point3d> Find3dPos();


};



void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
cv::Point FindACenter(std::vector<cv::Point> &geometry);


std::pair<int,int> FindMaxValue(cv::Mat &matrix, cv::Point &point );
/**Function: FindMaxValue
	*input: mat of scene, point of interest
	*output: two integrer value
	*Descriptio: function that calculates the max value of width and height for the roi 
*/


Point Camera::pos_object;
int Camera::press_buttom = 0;
int Camera::first_Step = 1;

// double Media(cv::Mat triangulatedPoints3D,double RobotLenght);
double Media(cv::Mat triangulatedPoints3D, double MaxLenght, int col);


void FromCvPointToEigen(cv::Point3d point_, Eigen::VectorXd &vect);
void FromMatToEigen(cv::Mat Mat_, Eigen::MatrixXd &Eigen);
void FromEigenVectorToCvPOint(Eigen::VectorXd Eigen, cv::Point3d &mat);

double ScalaReturn(double ptam, double ptam_prev, double robot);









#endif