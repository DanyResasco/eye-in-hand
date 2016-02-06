#ifndef Pollini_project_h
#define Pollini_project_h


// #include <boost/thread/thread.hpp>
// #include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <utility>
#include <list>
#include <string>
//#include <math.h> 
#include <stdio.h>
#include <eigen3/Eigen/Dense>
//ros
//#include <ros/ros.h>

//opencv
// #include "opencv2/core.hpp"
// #include "opencv2/core/utility.hpp"
// #include "opencv2/core/ocl.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/features2d.hpp"
// #include "opencv2/calib3d.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/xfeatures2d.hpp"
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/features2d.hpp>
// // #include "opencv2/xfeatures2d.hpp"
// #include "opencv2/core/version.hpp"
// #include <opencv2/xfeatures2d/nonfree.hpp>
// #include <opencv2/xfeatures2d/nonfree.hpp>
// #include <opencv2/nonfree.hpp>
// #include <opencv2/core/core_c.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/flann/miniflann.hpp>
// #include <opencv2/imgproc/imgproc_c.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/video/video.hpp>
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/objdetect/objdetect.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/ml/ml.hpp>
// #include <opencv2/highgui/highgui_c.h>
// #include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include "cv_bridge/CvBridge.h"
// #include <linux/videodev2.h>
// #include <asm/types.h>  

//sift
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "opencv2/calib3d/calib3d.hpp"


using namespace cv;



class Camera
{
	public:
		ros::NodeHandle nh;
		double video_or_photo;
		//to convert ros msg in opencv mat
		ros::Subscriber image_sub_;
    	image_transport::Publisher image_pub_;
    	// cvres_out.encoding = enc::TYPE_32FC1;
    	// cv::Mat scene;



		cv::Mat scene;
		int arrived_cam = 0;;
		// cv::Mat scene2;
		// cv::Mat frame2;
		// cv::Mat Camera_Matrix;
		// cv::Mat Distortion_Coefficients;
		// cv::Mat perspective_transformation_matrix;
		// cv::Mat duplicate_scene;
		// cv::Mat objcet_recognition;
		// std::string CAMERA_ROBOT;
		// double frame_width;
		// double frame_height;
		// static Point pos_object;
		// static int press_buttom;
		// cv::Point pos_object_real;
		// int first_Step = 1;
		// int start = 0;
		// cv::Mat imgDisparity8U; //disparity map
		// float Depth;

		double threshold_sift;

		struct Obj 
		{
			std::vector<cv::Point> Bot_C; //contour
			cv::Point Center_;
			std::vector<KeyPoint> keyp_;
			cv::Mat descr_;
			cv::Mat figure_;
		} BottonCHosen;


		image_transport::ImageTransport it_;
  	   image_transport::Subscriber sub;
  	   std::string camera_topic_;
  	   static Point pos_object;
		static int press_buttom;
		static int first_Step ;
		double Finish = 1;
		bool move_camera_end;
		// sensor_msgs::cv_bridge bridge_;

		int start = 0;

		Camera();
		~Camera(){};
		void ControllCamera();
		void DetectWithSift(cv::Mat &frame);
		bool MoveCamera();

	private:
		
		static void CallBackFunc(int event, int x, int y, int flags, void* userdata);
		// void DuplicateScene(Mat &frame_t);
		void ShapeDetect();
		// int FindAMinDistanceButton(std::vector<cv::Point> &baricentro);
		std::pair<int, bool> FindAMinDistanceButton(std::vector<cv::Point> &baricentro, cv::Point &point_);
		
		// void GetDisparityMap(cv::Mat &frame_cv);
		void ImageConverter(const sensor_msgs::ImageConstPtr& msg);
		std::pair<std::vector<cv::Point> ,std::vector<std::vector<cv::Point>> > FindContours(cv::Mat bw, cv::Mat camera);
		// Camera()
		// {	

		// 	// pos_object_real = pos_object;
		// 	//ros::param::get("~Data_point", Data_point);
		// }




	

};



void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
cv::Point FindACenter(std::vector<cv::Point> &geometry);

/**Function: FindMaxValue
	*input: mat of scene, point of interest
	*output: two integrer value
	*Descriptio: function that calculates the max value of width and height for the roi 
*/
std::pair<int,int> FindMaxValue(cv::Mat &matrix, cv::Point &point );



Point Camera::pos_object;
int Camera::press_buttom = 0;
int Camera::first_Step = 1;
























#endif