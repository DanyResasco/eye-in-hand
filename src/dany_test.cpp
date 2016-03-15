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
			if (node.first_Step == 1)
			{			    			
    			// imshow("CAMERA_ROBOT",  node.scene);
    			node.ControllCamera();
    			// waitKey(0);
    		}
    		// ROS_INFO_STREAM("ESCO DAL FIRST STEP");
			if(node.move_camera_end == true)
			{
				ROS_INFO_STREAM("entrata qui");
				// imshow("CAMERA_ROBOT_MOVE",  node.scene);
				node.DetectWithSift();
				// waitKey(0);
				if(node.FirstCalibration == 1)
				{
					node.StereoCalibration();
				}
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
	float cam_fx, cam_d0, cam_d1, cam_d2, cam_d3,cam_d4, cam_fy, cam_cx, cam_cy;
	nh.param<float>("/Camera_demo/cam_d0",cam_d0, -0.097079);
	nh.param<float>("/Camera_demo/cam_d1",cam_d1, 0.115189);
	nh.param<float>("/Camera_demo/cam_d2",cam_d2,-0.005712);
	nh.param<float>("/Camera_demo/cam_d3",cam_d3, 0.000509);
	nh.param<float>("/Camera_demo/cam_d4",cam_d4, 0.000509);

	nh.param<float>("/Camera_demo/cam_fx",cam_fx, 0);
	nh.param<float>("/Camera_demo/cam_fy",cam_fy, 0);
	nh.param<float>("/Camera_demo/cam_cx",cam_cx, 0);
	nh.param<float>("/Camera_demo/cam_cy",cam_cy, 0);

	nh.param<double>("/Camera_demo/RobotArmLenght",RobotArmLenght, 1);
	nh.param<double>("/Camera_demo/scale_factor", scale_factor,2.0);
	nh.param<double>("/Camera_demo/distanzaWebcam", distanzaWebcam,2.0);

	ROS_INFO_STREAM("distanzaWebcam: " <<distanzaWebcam);

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

 	ROS_INFO_STREAM("CAM ne lcostruttore: " << Camera_Matrix);

 	Cam_par_distortion = cv::Mat(1,5,CV_32FC1,0.0f);
 	Cam_par_distortion.at<float>(0,0) = cam_d0;
 	Cam_par_distortion.at<float>(0,1) = cam_d1;
 	Cam_par_distortion.at<float>(0,2) = cam_d2;
 	Cam_par_distortion.at<float>(0,3) = cam_d3; 
 	Cam_par_distortion.at<float>(0,3) = cam_d4; 



	move_camera_end = false;
	sub_ptam = false;
	read_ptam = false;
	FirstCalibration = 1;
	So3_new = 0;
	sub_ptam_2 = true;
	SaveFirst = true;



	sub = it_.subscribe("/camera/output_video", 1, &Camera::ImageConverter, this);
	ptam_sub = nh.subscribe("/vslam/pose",1, &Camera::SOtreCamera, this);  
	srv_move = nh.subscribe("/moveok",1, &Camera::MoveCallBack, this);
}


void Camera::ImageConverter(const sensor_msgs::Image::ConstPtr& msg)
{
	cv::Mat scene_temp;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv_ptr->image.copyTo(scene_temp);
    cv::undistort(scene_temp, scene, Camera_Matrix, Cam_par_distortion);



	arrived_cam = 1;
}


void Camera::ControllCamera()
{
	imshow("CAMERA_ROBOT",  scene);

 	/*set the callback function for any mouse event*/
	setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);
		waitKey(0);

	if(press_buttom == 1 )	/*wait the mouse event*/
	{
		ShapeDetect();
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

    /* Convert it to gray*/
  	cvtColor( scene, dst, CV_GRAY2RGB );
  	//Im1 = dst.clone();

  	
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
	
	Camera2_S03.at<float>(0,3) = frame_temp.p.x();
	Camera2_S03.at<float>(1,0) = frame_temp.M.data[3];
	Camera2_S03.at<float>(1,1) = frame_temp.M.data[4];
	Camera2_S03.at<float>(1,2) = frame_temp.M.data[5];

	Camera2_S03.at<float>(1,3) = frame_temp.p.y();
	Camera2_S03.at<float>(2,0) = frame_temp.M.data[6];
	Camera2_S03.at<float>(2,1) = frame_temp.M.data[7];
	Camera2_S03.at<float>(2,2) = frame_temp.M.data[8];

	Camera2_S03.at<float>(2,3) = frame_temp.p.z();

	if((sub_ptam == true) && (sub_ptam_2 == true))
    {
    	Triangulation(Camera2_S03);
    }
}


void Camera::DetectWithSift()
{
	ROS_INFO_STREAM("DENTRO DetectWithSift");
	// Im2 = frame.clone();
	cv::Mat frame;
	cv::Mat frame_temp;
	frame_temp = frame1_.clone();

	 	// cvtColor( frame1_, frame, CV_GRAY2RGB );
	frame_temp.convertTo(frame, CV_8U) ;
  	
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
	// cv::SiftFeatureDetector detector;
	// cv::SiftDescriptorExtractor extractor;


	detector.detect(frame, keyp_ );
	// cv::Mat img_keypoints_1;
	// drawKeypoints( frame, keyp_, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	// cv::imshow("key2", img_keypoints_1);
	// 		cv::waitKey(0);

	extractor.compute( frame, keyp_, descr_ );
 		
    /* -- Step 2: Matching descriptor vectors using FLANN matcher */
 	FlannBasedMatcher matcher;
 	std::vector< DMatch > matches;
    matcher.match( BottonCHosen.descr_, descr_, matches );

    double max_dist = 0; double min_dist = 100;

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
	 	if( matches[i].distance <= max(2*min_dist, 0.02) )
	    { 
		   	good_matches.push_back( matches[i]);

		   	KeypointIm2.push_back((keyp_[matches[i].trainIdx]).pt); 
	    }
    }

	/*-- Draw only "good" matches */
	Mat img_matches;
	drawMatches(BottonCHosen.figure_, BottonCHosen.keyp_, frame, keyp_, good_matches, img_matches, 
	  				Scalar::all(-1), Scalar::all(-1),std::vector<char>(), 
	  				DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	/*-- Localize the object */
	// std::vector<Point2f> obj;
	std::vector<Point> scene_point;
	ROS_INFO_STREAM(" good_matches.size(): " << good_matches.size());
	for(unsigned int i = 0; i < good_matches.size(); i++ )
	{
	    //-- Get the keypoints from the good matches
	    KeyPointIm1Match.push_back( BottonCHosen.keyp_[ good_matches[i].queryIdx ].pt );
	    scene_point.push_back( keyp_[ good_matches[i].trainIdx ].pt );
	}

	//test roi
	// cv::Point centerim2 = FindACenter(scene_point);
	// ROS_INFO_STREAM("CREATO FindACenter");
	// std::pair<int,int> value = FindMaxValue(frame, centerim2 );
	// cv::Mat roi(scene, Rect(centerim2.x - 30,centerim2.y - 40,value.first, value.second));
	// cv::Mat convert_BcMat_ = roi.clone(); 
	

	// convert_BcMat_.convertTo(Image_roi2, CV_8U) ;
	// 	//-- Show detected matches
	imshow( "good_matches", img_matches );
	waitKey(0);

	// ROS_INFO_STREAM("scene_point.SIZE()"<< scene_point.size());
	// ROS_INFO_STREAM("BottonCHosen.keyp_: " <<BottonCHosen.keyp_.size());
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
}


void Camera::StereoCalibration()
{

	ROS_INFO("Triangulation");
	cv::Mat key_array_1;
	cv::Mat key_array_2;

 	CreateAVector(KeypointIm2, KeyPointIm1Match , key_array_1, key_array_2);
 
  	Size imgSize = BottonCHosen.figure_.size();

	cv::Mat_<double> R = cv::Mat::eye(3, 3, CV_32FC1);
	cv::Mat_<double> T(3,1);
	T << distanzaWebcam,0.0,0.0;
	cv::Mat R1,P1,P2,Q; 
	cv::Mat R2 = cv::Mat(3,3,CV_32FC1,0.0f);
	
	cv::stereoRectify(Camera_Matrix, Cam_par_distortion, Camera_Matrix,Cam_par_distortion, imgSize,  R,  T,  R1,  R2,  P1,  P2,  Q);
	
	// ROS_INFO_STREAM("R1 DET: " << determinant(R1));
	// ROS_INFO_STREAM("R2 DET: " << determinant(R2));
	cv::Mat points4D;
	cv::triangulatePoints(P1, P2, key_array_1, key_array_2, points4D);
	// ROS_INFO_STREAM("points4D: " << points4D.size());
	if(points4D.cols > 3 )
	{
		cv::transpose(points4D, triangulatedPoints3D);
		cv::convertPointsFromHomogeneous(triangulatedPoints3D, triangulatedPoints3D);
		// media_z = scale_factor * Media(triangulatedPoints3D, RobotArmLenght);
		media_z = scale_factor * Media(triangulatedPoints3D, RobotArmLenght,2);
		ROS_INFO_STREAM("MEDIA * scale_factor su z: "<< media_z);

		// media_x =  Media(triangulatedPoints3D, 1.0, 0);
		// media_y =  Media(triangulatedPoints3D, 1.0, 1);
		FindXeY( Camera_Matrix, media_z, T);
		// ROS_INFO_STREAM("MEDIA * scale_factor su z: "<< media_z);
		// ROS_INFO_STREAM("MEDIA su y: "<< media_y);
		// ROS_INFO_STREAM("MEDIA su x: "<< media_x);
		FirstCalibration = 0;
		sub_ptam = true;
	}
	
}

void Camera::FindXeY(cv::Mat cameraMatrix, double media_z, cv::Mat tvec)
{
	cv::Mat uvPoint = cv::Mat::ones(3,1,CV_64F); //u,v,1
	uvPoint.at<double>(0,0) = BottonCHosen.Center_.x; //got this point using mouse callback
	uvPoint.at<double>(1,0) = BottonCHosen.Center_.y;

	double s;
	Eigen::Matrix3d Reigen = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d CamEigen;

	for(unsigned int i=0; i < cameraMatrix.rows; i++)
	{
		for(unsigned int j =0; j < cameraMatrix.cols; j++)
		{
			CamEigen(i,j) = cameraMatrix.at<float>(i,j);
		}
	}

	
	Eigen::Vector3d Point2D;
	Eigen::Vector3d T;
	Point2D[0] = uvPoint.at<double>(0,0);
	Point2D[1] = uvPoint.at<double>(1,0);
	Point2D[2] = uvPoint.at<double>(2,0);
	T[0] =  tvec.at<double>(0,0);
	T[1] =  tvec.at<double>(1,0);
	T[2] =  tvec.at<double>(2,0);
	
	
	Eigen::Vector3d prod_eigen;
	prod_eigen = Reigen.inverse()*CamEigen.inverse()*Point2D;

	Eigen::Vector3d temp2;
	temp2 = Reigen.inverse()*T;
	s = (media_z + temp2(2,0))/prod_eigen(2,0);
	Eigen::Vector3d p;
	p = Reigen.inverse()*(s*CamEigen.inverse()*Point2D - T);

	ROS_INFO_STREAM("POINT 2d: " << p);
	// Point3dTriangulate.at<double>(0,0) = p[0];
	// Point3dTriangulate.at<double>(0,1) = p[1];
	// Point3dTriangulate.at<double>(0,2) = p[2];
	// ROS_INFO_STREAM("Point3dTriangulate: " << Point3dTriangulate);
}

double Media(cv::Mat triangulatedPoints3D, double MaxLenght, int col)
{
	float temp = 0;
	float count = 0;

	for(unsigned int i=0; i< triangulatedPoints3D.rows; i++)
	{	
		if( std::abs(triangulatedPoints3D.at<float>(i,col)) <= MaxLenght  )
		{
			temp = temp + std::abs(triangulatedPoints3D.at<float>(i,col)); 
			count = count + 1;
		}
	}

	double media = temp/count;
	
	return media;
}
















// double Media(cv::Mat triangulatedPoints3D, double RobotLenght)
// {
// 	float temp = 0;
// 	float count = 0;

// 	for(unsigned int i=0; i< triangulatedPoints3D.rows; i++)
// 	{	
// 		if( std::abs(triangulatedPoints3D.at<float>(i,2)) <= RobotLenght  )
// 		{
// 			temp = temp + std::abs(triangulatedPoints3D.at<float>(i,2)); 
// 			count = count + 1;
// 		}
// 	}

// 	double media_z = temp/count;
	
// 	return media_z;
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

	move_camera_end = msg->data;
	frame1_ = scene.clone();
	if(frame1_.empty())
	{
		ROS_ERROR("empty mat frame");
		// return 0;
	}

	sub_ptam_2 = true;
}



void Camera::Triangulation(cv::Mat S03_ptam)
{
	double distance_z;
	double distance_x;
	double distance_y;
	
	if(SaveFirst == true)
	{
		So3_prev = std::abs(S03_ptam.at<float>(2,3));
		distance_z = media_z;
		SaveFirst = false;
		// scala = So3_prev/distance_z;
	}
	else
	{
		if(So3_prev < std::abs(S03_ptam.at<float>(2,3)))
		{
			ROS_INFO("pTAM da i numeri.. trova una soluzione");
		}
		else
		{
		 	So3_new = So3_new + So3_prev - std::abs(S03_ptam.at<float>(2,3));

		 	// ROS_INFO_STREAM("So3_new. " << So3_new);
		 	// ROS_INFO_STREAM("So3_prev: " << So3_prev);
		 	distance_z =  media_z - So3_new;
		 	// distance_x = std::abs(S03_ptam.at<float>(2,1)) / scala;
		 	// distance_y = std::abs(S03_ptam.at<float>(2,2)) / scala;
		 	So3_prev = std::abs(S03_ptam.at<float>(2,3)); 
		 	// ROS_INFO_STREAM("La webcam su z e' in posizione :" << distance_z); 
		 	// ROS_INFO_STREAM("La webcam su y e' in posizione :" << distance_y); 
		 	// ROS_INFO_STREAM("La webcam su x e' in posizione :" << distance_x); 
		}
	}
	sub_ptam_2 = false;
}
