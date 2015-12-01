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
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
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





using namespace cv;



class Camera
{
	private:
		// ros::NodeHandle nodeH;
		cv::Mat scene;
		// Mat frame_;
		cv::Mat duplicate_scene;
		cv::Mat objcet_recognition;
		std::string CAMERA_ROBOT;
		double frame_width;
		double frame_height;
		static Point pos_object;
		static int press_buttom;
		cv::Point pos_object_real;
		std::vector<KeyPoint> kp1;
		cv::Mat des1;
		int first_Step = 1;
		Mat disparity;
		int start = 0;
		cv::Mat imgDisparity8U; //disparity map
		double Depth;

		struct Obj 
		{
			std::vector<cv::Point> Bot_C; //contour
			cv::Point Center_;
			std::vector<KeyPoint> keyp_;
			cv::Mat descr_;
			cv::Mat figure_;
		} BottonCHosen;



	public:
		void ControllCamera();
		static void CallBackFunc(int event, int x, int y, int flags, void* userdata);
		void DuplicateScene(Mat &frame_t);
		void ShapeDetect();
		std::pair<int, bool> FindAMinDistanceButton(std::vector<cv::Point> &baricentro);
		void DetectAndMove(cv::Mat &frame);
		void GetDisparityMap(cv::Mat &frame_cv);
		Camera()
		{	

			// pos_object_real = pos_object;
			//ros::param::get("~Data_point", Data_point);
		}




	~Camera(){};

};



void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
cv::Point FindACenter(std::vector<cv::Point> &geometry);



Point Camera::pos_object;
int Camera::press_buttom = 0;
























#endif