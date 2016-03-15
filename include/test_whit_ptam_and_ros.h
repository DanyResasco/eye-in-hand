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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Pose.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>

//sift
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

// #include "opencv2/imgcodecs.hpp"

// #include "opencv2/core/utility.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <tf_conversions/tf_kdl.h>
#include <std_srvs/Empty.h>



using namespace cv;



class Camera
{
	public:
		ros::NodeHandle nh;
		// double video_or_photo;
    	image_transport::Publisher image_pub_;
    	ros::Subscriber ptam_sub;
    	cv::Mat Camera_Matrix;
    	cv::Mat Cam_par_distortion;
    	cv::Mat Camera2_S03;
    	cv::Mat Im1;
    	cv::Mat Im2;

		cv::Mat scene;
		int arrived_cam = 0;
		double ptam_scale;
		bool sub_ptam;
		cv::Mat mat_;
		ros::Subscriber srv_move;
	
		struct Obj 
		{
			std::vector<cv::Point> Bot_C; //contour
			cv::Point Center_;
			std::vector<KeyPoint> keyp_;
			cv::Mat descr_;
			cv::Mat figure_;
		} BottonCHosen;

		cv::Mat Point_3d; 
		cv::Point Botton_2frame;

		std::vector<cv::Point2f> KeypointIm2;
		std::vector<Point2f> KeyPointIm1Match;
		cv::Mat triangulatedPoints3D;

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
		double scale_factor;
		double distanzaWebcam;
		double So3_prev, So3_new;
		bool SaveFirst;
		bool sub_ptam_2;
		double scala;
		cv::Mat Point3dTriangulate;

		Camera();
		~Camera(){};
		void ControllCamera();
		void DetectWithSift();
		void StereoCalibration();
		

	private:
		
		static void CallBackFunc(int event, int x, int y, int flags, void* userdata);
		void ShapeDetect();
		std::pair<int, bool> FindAMinDistanceButton(std::vector<cv::Point> &baricentro, cv::Point &point_);
		void ImageConverter(const sensor_msgs::Image::ConstPtr& msg);
		std::pair<std::vector<cv::Point> ,std::vector<std::vector<cv::Point>> > FindContours(cv::Mat bw, cv::Mat camera);
		void SOtreCamera(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
		void Triangulation(cv::Mat S03_ptam);
		void CreateAVector(std::vector<Point2f> keyp2, std::vector<Point2f> keyp_1 , cv::Mat &key_array_1, cv::Mat &key_array_2);
		void MoveCallBack(const std_msgs::Bool::ConstPtr msg);
		void FindXeY(cv::Mat cameraMatrix, double media_z, cv::Mat tvec);
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



















#endif