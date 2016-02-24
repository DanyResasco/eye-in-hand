// using namespace std;
#include <test_whit_ptam_and_ros.h>
#include <function_camera.hpp>

using namespace cv;
RNG rng(12345);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "eye_in_hand_node");
	Camera node;
	
	ROS_INFO("[eye_in_hand] Node is ready");

	double spin_rate = 50;
	ros::param::get("~spin_rate",spin_rate);
	
	ROS_DEBUG( "Spin Rate %lf", spin_rate);

	namedWindow("CAMERA_ROBOT");

	ros::Rate rate(spin_rate); 
		
	while ( ros::ok() )
    {
    	/*if camera ok*/
        if(node.arrived_cam ==1)
        {            	
    		if(node.Finish == 1)
    		{
    			imshow("CAMERA_ROBOT",  node.scene);
    			node.ControllCamera();
    			waitKey(0);
    		}


    		
    	}
            
        //relax to fit output rate
        rate.sleep(); 
        ros::spinOnce();            
    }

	return 0;
}

Camera::Camera(): it_(nh)
{
	// ros::param::get("~video_or_photo", video_or_photo);
	// ROS_DEBUG( "video_or_photo %lf", video_or_photo);
	ROS_INFO("COSTRUTTORE NODE DANY_TEST");
	nh.param<double>("ptam_scale",ptam_scale, 0);

	/*Set the camera parameter from calibration. Change this parameter at code_param.yaml */
	double cam_fx, cam_d0, cam_d1, cam_d2, cam_d3, cam_fy, cam_cx, cam_cy;
	nh.param<double>("/Camera_demo/cam_d0",cam_d0, -0.097079);
	nh.param<double>("/Camera_demo/cam_d1",cam_d1, 0.115189);
	nh.param<double>("/Camera_demo/cam_d2",cam_d2,-0.005712);
	nh.param<double>("/Camera_demo/cam_d3",cam_d3, 0.000509);

	nh.param<double>("/Camera_demo/cam_fx",cam_fx, 0);
	nh.param<double>("/Camera_demo/cam_fy",cam_fy, 0);
	nh.param<double>("/Camera_demo/cam_cx",cam_cx, 0);
	nh.param<double>("/Camera_demo/cam_cy",cam_cy, 0);

	Camera_Matrix = cv::Mat(3,3,CV_32FC1,0.0f);
	Camera2_S03 = cv::Mat(3,3,CV_32FC1,0.0f);

 	Camera_Matrix.at<float>(0,0) = cam_fx;
 	Camera_Matrix.at<float>(0,1) = 0;
 	Camera_Matrix.at<float>(0,2) = cam_cx;
 	Camera_Matrix.at<float>(1,0) = 0;
 	Camera_Matrix.at<float>(1,1) = cam_fy;
 	Camera_Matrix.at<float>(1,2) = cam_cy;
 	Camera_Matrix.at<float>(2,0) = 0;
 	Camera_Matrix.at<float>(2,1) = 0;
 	Camera_Matrix.at<float>(2,2) = 1;

 	Cam_par_distortion = cv::Mat(1,4,CV_32FC1,0.0f);
 	Cam_par_distortion.at<float>(0,0) = cam_d0;
 	Cam_par_distortion.at<float>(0,1) = cam_d1;
 	Cam_par_distortion.at<float>(0,2) = cam_d2;
 	Cam_par_distortion.at<float>(0,3) = cam_d3; 

	sub = it_.subscribe("/camera/output_video", 1, &Camera::ImageConverter, this);
	move_camera_end = false;

	ptam_sub = nh.subscribe("/vslam/pose",1, &Camera::SOtreCamera, this);
}


void Camera::ImageConverter(const sensor_msgs::Image::ConstPtr& msg)
{
	cv::Mat scene_temp;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv_ptr->image.copyTo(scene_temp);

    cv::undistort(scene_temp, scene, Camera_Matrix, Cam_par_distortion);

	arrived_cam = 1;
	ROS_INFO("RICEVUTA CAMERA");
}


void Camera::ControllCamera()
{
	if (first_Step == 1)
	{
	 	/*set the callback function for any mouse event*/
		setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);

		if(press_buttom == 1 )	/*wait the mouse event*/
		{
			ShapeDetect();
		}
	}
	std::vector< DMatch > keyp2;
	if(move_camera_end == true)
    {
    	keyp2 = DetectWithSift(scene);
    }
    if(sub_ptam == true)
    {
    	Triangulation(keyp2);
    }
    		

}

void Camera::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if ( event == EVENT_LBUTTONDOWN )
    {
       	std::cout << "Left mouse button is clicked. Save the position (" << x << ", " << y << ")" <<std::endl;
        pos_object.x = x;
    	pos_object.y = y;
   		press_buttom = 1;
    }

}



void Camera::ShapeDetect()
{ 
	Mat src_gray;
	cv::Mat dst ;
	//= scene.clone();
    /* Convert it to gray*/
  	cvtColor( scene, dst, CV_GRAY2RGB );

  	/* Reduce the noise so we avoid false circle detection*/
  	GaussianBlur( scene, src_gray, Size(9, 9), 2, 2 );
  	
  	/* Convert to binary image using Canny*/
	cv::Mat bw;
	cv::Canny(scene, bw, 0, 50, 5);

	std::pair<std::vector<cv::Point> ,std::vector<std::vector<cv::Point> >> CenterAndContours;
	
	CenterAndContours = FindContours(bw.clone(), dst.clone());
	ROS_INFO("DENTRO ShapeDetect");
	if(CenterAndContours.second.size() > 0)
	{
		std::pair<int,bool> info_geometry;
		cv::Point point_b;
		point_b.x = pos_object.x;
		point_b.y = pos_object.y;

		info_geometry = FindAMinDistanceButton(CenterAndContours.first,point_b);
		
		if(info_geometry.second == true)
		{
			ROS_INFO("** Tasto premuto correttamente **");

			BottonCHosen.Center_.x = floor(CenterAndContours.first[info_geometry.first].x);
			BottonCHosen.Center_.y = floor(CenterAndContours.first[info_geometry.first].y);
			setLabel(dst, "BOTP", CenterAndContours.second[info_geometry.first]);
			BottonCHosen.Bot_C = CenterAndContours.second[info_geometry.first];

			std::pair<int,int> value = FindMaxValue(dst, BottonCHosen.Center_ );
			cv::Mat roi(scene, Rect(BottonCHosen.Center_.x - 30,BottonCHosen.Center_.y - 40,value.first, value.second));
			cv::Mat convert_BcMat_ = roi.clone(); 

			convert_BcMat_.convertTo(BottonCHosen.figure_, CV_8U) ;
	
		    /* SIFT feature detector and feature extractor */
		    cv::SiftFeatureDetector detector( 0.01, 3.0 );
		    cv::SiftDescriptorExtractor extractor( 2.0 );

		    detector.detect(BottonCHosen.figure_, BottonCHosen.keyp_ );
		    extractor.compute( BottonCHosen.figure_, BottonCHosen.keyp_, BottonCHosen.descr_ );

			cv::imshow("dst", dst);
			cv::waitKey(0);

			Finish = 0;
			move_camera_end = true;
			first_Step = 0;
		}
		else
		{
			press_buttom = 0;
			ROS_INFO("ripremi nuovamente il pulsante");
		}

	}
	else
	{
		ROS_INFO("ripremi nuovamente il pulsante");
		press_buttom = 0;
	}		
}


void Camera::SOtreCamera(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
	KDL::Frame frame_temp;
	
	tf::poseMsgToKDL ((msg->pose).pose, frame_temp);
	ROS_INFO_STREAM("frame_temp: "<<frame_temp);
	Camera2_S03.at<float>(0,0) = frame_temp.M.data[0];
	Camera2_S03.at<float>(0,1) = frame_temp.M.data[1];
	Camera2_S03.at<float>(0,2) = frame_temp.M.data[2];
	Camera2_S03.at<float>(0,3) = frame_temp.p.x();
	Camera2_S03.at<float>(1,0) = frame_temp.M.data[3];
	Camera2_S03.at<float>(1,1) = frame_temp.M.data[4];
	Camera2_S03.at<float>(1,2) = frame_temp.M.data[5];
	Camera2_S03.at<float>(1,3) = frame_temp.p.y();
	Camera2_S03.at<float>(2,0) = frame_temp.M.data[6];
	Camera2_S03.at<float>(2,1) = frame_temp.M.data[7];
	Camera2_S03.at<float>(2,2) = frame_temp.M.data[8];
	Camera2_S03.at<float>(2,3) = frame_temp.p.z();
}


std::vector< DMatch > Camera::DetectWithSift(cv::Mat &frame)
{
	//Detect sift
		 /* threshold      = 0.04;
       		edge_threshold = 10.0;
       		magnification  = 3.0;    */ 
	// SIFT feature detector and feature extractor
    // std::cout<<"BottonCHosen.descr_.rows: "<<BottonCHosen.descr_.rows<<std::endl;
    std::vector<KeyPoint> keyp_;
	cv::Mat descr_;   		
	cv::SiftFeatureDetector detector( 0.01, 3.0 );
	cv::SiftDescriptorExtractor extractor( 2.0 );

	detector.detect(frame, keyp_ );
	extractor.compute( frame, keyp_, descr_ );
 	
    /* -- Step 2: Matching descriptor vectors using FLANN matcher */
 	FlannBasedMatcher matcher;
 	std::vector< DMatch > matches;
    matcher.match( BottonCHosen.descr_, descr_, matches );

    double max_dist = 0; double min_dist = 70;

	/*-- Quick calculation of max and min distances between keypoints */
	for( int i = 0; i < BottonCHosen.descr_.rows; i++ )
	{ 
	 	double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	}

	std::vector< DMatch > good_matches;
	std::vector<cv::Point> pointIm2;

	for( int i = 0; i < BottonCHosen.descr_.rows; i++ )
	{ 
	 	if( matches[i].distance <= max(min_dist, 0.02) )
	    { 
		   	good_matches.push_back( matches[i]);

		   	pointIm2.push_back((keyp_[matches[i].trainIdx]).pt); 
		   	
	    }
    }

	/*-- Draw only "good" matches */
	Mat img_matches;
	drawMatches( BottonCHosen.figure_, BottonCHosen.keyp_, frame, keyp_, good_matches, img_matches, 
	  				Scalar::all(-1), Scalar::all(-1),std::vector<char>(), 
	  				DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	/*-- Localize the object */
	std::vector<Point2f> obj;
	std::vector<Point> scene_point;

	for(unsigned int i = 0; i < good_matches.size(); i++ )
	{
	    //-- Get the keypoints from the good matches
	    obj.push_back( BottonCHosen.keyp_[ good_matches[i].queryIdx ].pt );
	    scene_point.push_back( keyp_[ good_matches[i].trainIdx ].pt );
	}
	if(scene_point.size() >0)
	{
		setLabel(frame, "quip", scene_point);
		cv::Rect r = cv::boundingRect(scene_point);
		cv::Point pt(r.x + (r.width / 2), r.y + (r.height / 2));
		Botton_2frame = pt;
		line( frame,pt, pt , Scalar( 110, 220, 0 ),  2, 8 );
		imshow("Object detection",frame);
		waitKey(0);
	}

    sub_ptam = true;

    return good_matches;
	//-- Show detected matches
	// imshow( "Good Matches", img_matches );
	// waitKey(0);
}


void Camera::Triangulation(std::vector< DMatch > keyp2)
{

	cv::Mat points3D(4,1,CV_64FC4);
// cv::Mat cam0pnts(1,N,CV_64FC2);
// cv::Mat cam1pnts(1,N,CV_64FC2);

	cv::Mat projMatr1 = Camera_Matrix;
	cv::Mat projMatr2 = Camera_Matrix * Camera2_S03;

	cv::triangulatePoints(projMatr1, projMatr2, BottonCHosen.keyp_, keyp2, points3D);



}

