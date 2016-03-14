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
	double cam_fx, cam_d0, cam_d1, cam_d2, cam_d3,cam_d4, cam_fy, cam_cx, cam_cy;
	nh.param<double>("/Camera_demo/cam_d0",cam_d0, -0.097079);
	nh.param<double>("/Camera_demo/cam_d1",cam_d1, 0.115189);
	nh.param<double>("/Camera_demo/cam_d2",cam_d2,-0.005712);
	nh.param<double>("/Camera_demo/cam_d3",cam_d3, 0.000509);
	nh.param<double>("/Camera_demo/cam_d4",cam_d4, 0.000509);

	nh.param<double>("/Camera_demo/cam_fx",cam_fx, 0);
	nh.param<double>("/Camera_demo/cam_fy",cam_fy, 0);
	nh.param<double>("/Camera_demo/cam_cx",cam_cx, 0);
	nh.param<double>("/Camera_demo/cam_cy",cam_cy, 0);

	Camera_Matrix = cv::Mat(3,3,CV_32FC1,0.0f);
	Camera2_S03 = cv::Mat(3,4,CV_32FC1,0.0f);

 	Camera_Matrix.at<float>(0,0) = cam_fx;
 	Camera_Matrix.at<float>(0,1) = 0;
 	Camera_Matrix.at<float>(0,2) = cam_cx;
 	Camera_Matrix.at<float>(1,0) = 0;
 	Camera_Matrix.at<float>(1,1) = cam_fy;
 	Camera_Matrix.at<float>(1,2) = cam_cy;
 	Camera_Matrix.at<float>(2,0) = 0;
 	Camera_Matrix.at<float>(2,1) = 0;
 	Camera_Matrix.at<float>(2,2) = 1;

 	Cam_par_distortion = cv::Mat(1,5,CV_32FC1,0.0f);
 	Cam_par_distortion.at<float>(0,0) = cam_d0;
 	Cam_par_distortion.at<float>(0,1) = cam_d1;
 	Cam_par_distortion.at<float>(0,2) = cam_d2;
 	Cam_par_distortion.at<float>(0,3) = cam_d3; 
 	Cam_par_distortion.at<float>(0,3) = cam_d4; 

	sub = it_.subscribe("/camera/output_video", 1, &Camera::ImageConverter, this);
	move_camera_end = false;
	sub_ptam = false;
	read_ptam = false;
	ptam_sub = nh.subscribe("/vslam/pose",1, &Camera::SOtreCamera, this);  
	srv_move = nh.subscribe("/moveok",1, &Camera::MoveCallBack, this);

}


void Camera::ImageConverter(const sensor_msgs::Image::ConstPtr& msg)
{
	cv::Mat scene_temp;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv_ptr->image.copyTo(scene_temp);
      // cv_ptr->image.copyTo(scene);
    cv::undistort(scene_temp, scene, Camera_Matrix, Cam_par_distortion);

	arrived_cam = 1;
	// ROS_INFO("RICEVUTA CAMERA");
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
    // ROS_INFO_STREAM("sub_ptam: " <<std::boolalpha<<sub_ptam);
    // ROS_INFO_STREAM("read_ptam: " <<std::boolalpha<<read_ptam);

    // if(sub_ptam == true) 
    // {
    // 	Triangulation();
    // }
    		

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
	Im1 = scene.clone();
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
			// cv::imshow("dst_tagliato", BottonCHosen.figure_);
			// cv::waitKey(0);

			Finish = 0;
			// move_camera_end = true;
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
	// ROS_INFO("LEGGO TOPIC");
	tf::poseMsgToKDL ((msg->pose).pose, frame_temp);
	// ROS_INFO_STREAM("frame_temp: "<<frame_temp);
	Camera2_S03.at<float>(0,0) = frame_temp.M.data[0];
	Camera2_S03.at<float>(0,1) = frame_temp.M.data[1];
	Camera2_S03.at<float>(0,2) = frame_temp.M.data[2];
	Camera2_S03.at<float>(0,3) = 0;
	// Camera2_S03.at<float>(0,3) = frame_temp.p.x();
	Camera2_S03.at<float>(1,0) = frame_temp.M.data[3];
	Camera2_S03.at<float>(1,1) = frame_temp.M.data[4];
	Camera2_S03.at<float>(1,2) = frame_temp.M.data[5];
		Camera2_S03.at<float>(1,3) = 0;
	// Camera2_S03.at<float>(1,3) = frame_temp.p.y();
	Camera2_S03.at<float>(2,0) = frame_temp.M.data[6];
	Camera2_S03.at<float>(2,1) = frame_temp.M.data[7];
	Camera2_S03.at<float>(2,2) = frame_temp.M.data[8];
		Camera2_S03.at<float>(2,3) = 0.05;
	// Camera2_S03.at<float>(2,3) = frame_temp.p.z();

	// ROS_INFO_STREAM("Camera2_S03: "<<Camera2_S03);
	// read_ptam = true;
	if(sub_ptam == true)
    {
    	Triangulation();
    }
}


std::vector< DMatch > Camera::DetectWithSift(cv::Mat &frame)
{
	Im2 = frame.clone();
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

    double max_dist = 0; double min_dist = 55;

	/*-- Quick calculation of max and min distances between keypoints */
	for( int i = 0; i < BottonCHosen.descr_.rows; i++ )
	{ 
	 	double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	}

	std::vector< DMatch > good_matches;
	// std::vector<cv::Point2f> pointIm2;

	for( int i = 0; i < BottonCHosen.descr_.rows; i++ )
	{ 
	 	if( matches[i].distance <= max(min_dist, 0.01) )
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
	// std::vector<Point2f> obj;
	std::vector<Point> scene_point;

	for(unsigned int i = 0; i < good_matches.size(); i++ )
	{
	    //-- Get the keypoints from the good matches
	    obj.push_back( BottonCHosen.keyp_[ good_matches[i].queryIdx ].pt );
	    scene_point.push_back( keyp_[ good_matches[i].trainIdx ].pt );
	}

	//test roi
	cv::Point centerim2 = FindACenter(scene_point);
	std::pair<int,int> value = FindMaxValue(frame, centerim2 );
	cv::Mat roi(scene, Rect(centerim2.x - 30,centerim2.y - 40,value.first, value.second));
	cv::Mat convert_BcMat_ = roi.clone(); 
	convert_BcMat_.convertTo(im2, CV_8U) ;




	// 	//-- Show detected matches
	// imshow( "roi2", im2 );
	// waitKey(0);

	ROS_INFO_STREAM("scene_point.SIZE()"<< scene_point.size());
	ROS_INFO_STREAM("BottonCHosen.keyp_: " <<BottonCHosen.keyp_.size());
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
}


void Camera::Triangulation()
{

	ROS_INFO("Triangulation");
	cv::Mat key_array_1;
	cv::Mat key_array_2;

 	CreateAVector(pointIm2, obj , key_array_1, key_array_2);
 	int number = key_array_1.rows;

 	cv::Mat points4D(1,number,CV_32FC1);
 	std::vector<Point2f> points3D;
	
 	cv::Mat projMatr1;
 	projMatr1 = cv::Mat(3,4,CV_32FC1,0.0f);
 

 	for( int i=0; i<Camera_Matrix.rows; i++)
 	{
 		for( int j=0; j<Camera_Matrix.cols; j++)
 		{
 			projMatr1.at<float>(i,j) = Camera_Matrix.at<float>(i,j);
 		}
 	}
 	
 	projMatr1.at<float>(0,3) = 0;
 	projMatr1.at<float>(1,3) = 0;
 	projMatr1.at<float>(2,3) = 0;

 	cv::Mat projMatr2;
 	projMatr2 = cv::Mat(3,4,CV_32FC1,0.0f);
 	projMatr2 = projMatr1;

 	// Size imgSize = scene.size();
 		Size imgSize = BottonCHosen.figure_.size();

	cv::Mat_<double> R = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat_<double> T(3,1);
	T << 0.0,0,0.05;
	cv::Mat R1,R2,P1,P2,Q; 
	cv::stereoRectify(Camera_Matrix, Cam_par_distortion, Camera_Matrix,Cam_par_distortion, imgSize,  R,  T,  R1,  R2,  P1,  P2,  Q);
	// ROS_INFO_STREAM("P2:" << P2);

	// cv::Mat OR_scene;
	// cv::Mat new_frame;
	// cv::Mat imgDisparity8U ;
	// 	// imgDisparity8U = Mat( BottonCHosen.figure_.rows, BottonCHosen.figure_.cols, CV_8U );
	// cv::Mat imgDisparity16S;
	// //= Mat( scene.rows, scene.cols, CV_16S );
	// imgDisparity8U = Mat(im2.rows, im2.cols, CV_8U );
	// // ROS_INFO("FATTO 8U");
 //  	// std::cout<<"qui"<<std::endl;
 //  	// cvtColor( scene, new_Im2, CV_BGR2GRAY );
	// int ndisparities = 16*8;  // < Range of disparity 
 //  	int SADWindowSize = 50; //< Size of the block window. Must be odd 
	// cv::StereoBM StereoBM( ndisparities, SADWindowSize );

	// //  -- 3. Calculate the disparity image
	// ROS_INFO_STREAM("BottonCHosen.figure_" << BottonCHosen.figure_.size());
	// ROS_INFO_STREAM("scene" << im2.size());
	// StereoBM.operator()( BottonCHosen.figure_, im2, imgDisparity8U, CV_32F );
	// ROS_INFO_STREAM("QUI");
	// imgDisparity8U.convertTo(imgDisparity16S, CV_32F);

	// cv::Mat _3dImage;
	// cv::reprojectImageTo3D(imgDisparity8U, _3dImage,  Q, false, -1 );
	// ROS_INFO_STREAM("_3dImage: " <<_3dImage);




	cv::triangulatePoints(P1, P2, key_array_1, key_array_2, points4D);

	Mat triangulatedPoints3D;
	cv::transpose(points4D, triangulatedPoints3D);
	cv::convertPointsFromHomogeneous(triangulatedPoints3D, triangulatedPoints3D);
	ROS_INFO_STREAM("triangulatedPoints3D: " <<triangulatedPoints3D);
}

// void Media(cv::Mat triangulatedPoints3D)
// {
// 	double temp = 0;
// 	double count = 0;

// 	for(unsigned int i=0; i< triangulatedPoints3D.rows-1; i++)
// 	{	
// 		// double temp_distance = std::abs(triangulatedPoints3D.at<double>(i,2)) - std::abs(triangulatedPoints3D.at<double>(i+1,2)) ;
// 		// if( temp_distance < 0.35  )
// 		// {
// 			temp += std::abs(triangulatedPoints3D.at<double>(i,2)); 
// 			count +=1;
// 		// }
// 		// ROS_INFO_STREAM("temp: " << temp);
// 		// ROS_INFO_STREAM("triangulatedPoints3D.at<double> i: " << i <<"valore"<< triangulatedPoints3D.at<double>(i,2));
// 	}

// 	double media = temp/count;
// 	ROS_INFO_STREAM("MEDIA: "<< media);

// }


void Camera::CreateAVector(std::vector<Point2f> keyp2, std::vector<Point2f> keyp_1 , cv::Mat &key_array_1, cv::Mat &key_array_2)
{	
	std::vector<cv::Point2f> points;
	std::vector<cv::Point2f> points2;
	std::vector<Point2f> v_count;

	
	if(keyp2.size() >= keyp_1.size())
	{
		v_count.resize(keyp_1.size());
	}
	else
	{
		v_count.resize(keyp2.size());
	}

	for (unsigned int i=0; i< v_count.size(); i++)
	{
		points2.push_back(keyp2[i]);
		points.push_back(keyp_1[i]);
	}

	cv::Mat key_array_1_temp(points);
	key_array_1_temp.copyTo(key_array_1);
	cv::Mat key_array_2_temp(points2);
	key_array_2_temp.copyTo(key_array_2);

 	// // ROS_INFO_STREAM("test_TRANSPOSE" << )
 	// ROS_INFO_STREAM("key_array_1: "<< key_array_1.size());
 	// ROS_INFO_STREAM("key_array_2: "<< key_array_2.size());
}


void Camera::MoveCallBack(const std_msgs::Bool::ConstPtr msg)
{
	move_camera_end = msg->true;
}