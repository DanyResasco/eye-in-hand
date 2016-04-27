// using namespace std;
#include <Camera_code.h>
#include <geometry_function.hpp>
#include <converter_file.hpp>
#include <plane_estimate.hpp>
// #include <Scale_estimation.hpp>

// using namespace cv;
cv::RNG rng(12345);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "eye_in_hand_node");
	Camera node;
	
	ROS_INFO("[eye_in_hand] Node is ready");

	double spin_rate = 50;
	ros::param::get("~spin_rate",spin_rate);
	
	ROS_DEBUG( "Spin Rate %lf", spin_rate);

	cv::namedWindow("CAMERA_ROBOT");

	ros::Rate rate(spin_rate); 
		
	while ( ros::ok() )
    {
    	/*if camera ok*/
        if(node.arrived_cam ==1)
        {            	
			if (node.first_Step == 1)
			{			    			
    			node.ControllCamera();
    		}

			if(node.move_camera_end == true)
			{
				node.DetectWithSift();

				if(node.stop_flag == true)
				{
					node.Triangulation();
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
	/*Set the camera parameter from calibration. Change this parameter at code_param.yaml */
	nh.param<float>("/Camera_demo/cam_d0",cam_d0, -0.097079);
	nh.param<float>("/Camera_demo/cam_d1",cam_d1, 0.115189);
	nh.param<float>("/Camera_demo/cam_d2",cam_d2,-0.005712);
	nh.param<float>("/Camera_demo/cam_d3",cam_d3, 0.000509);
	nh.param<float>("/Camera_demo/cam_d4",cam_d4, 0.000509);

	nh.param<float>("/Camera_demo/cam_fx",cam_fx, 0);
	nh.param<float>("/Camera_demo/cam_fy",cam_fy, 0);
	nh.param<float>("/Camera_demo/cam_cx",cam_cx, 0);
	nh.param<float>("/Camera_demo/cam_cy",cam_cy, 0);

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

 	InfoKf3d_ = false;

 	//initialize the flag
	move_camera_end = false;
	// sub_ptam_2 = false;
	// count_n_passi = 0;
	stop_flag = false;
	
	scala = 1;
	myfile1.open("/home/daniela/code/src/eye_in_hand/pos_log.txt");
	myfile.open("/home/daniela/code/src/eye_in_hand/scale_log.txt");
	myfile4.open("/home/daniela/code/src/eye_in_hand/ptam_pose1.txt");

	sub = it_.subscribe("/camera/output_video", 1, &Camera::ImageConverter, this);
	ptam_sub = nh.subscribe("/vslam/pose",1, &Camera::SOtreCamera, this);  //word in camera framebu
	movewebcamrobot = nh.subscribe("/robot",1, &Camera::RobotMove,this); // robot in cam frame
	ptam_kf3d = nh.subscribe("/vslam/pc2",1,&Camera::InfoKf3d,this);	//point in word frame
	scala_sub = nh.subscribe("/scala_",1,&Camera::ScaleCallback,this);	//to stop the pc2 callback
}

void Camera::ScaleCallback(const std_msgs::Float32::ConstPtr msg)
{
	scala = msg->data;
	move_camera_end = true;
	stop_flag = true;
	frame1_ = scene.clone();
}


void Camera::InfoKf3d(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO_STREAM("qui InfoKf3d");
	//Converto da pointcloud2 a pcl::XYZ
	pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, Ptamkf3d);
    InfoKf3d_ = true;
}  	



void Camera::RobotMove(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO_STREAM("RICEVUTO Messaggio");
	stop_flag = msg->data;
	frame1_ = scene.clone(); 
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
	cv::imshow("CAMERA_ROBOT",  scene);
	
 	/*set the callback function for any mouse event*/
	cv::setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);
	cv::waitKey(0);

	if(press_buttom == 1 )	/*wait the mouse event*/
	{
		ShapeDetect();
	}
}

void Camera::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if ( event == cv::EVENT_LBUTTONDOWN )
    {
       	std::cout << "Left mouse button is clicked. Save the position (" << x << ", " << y << ")" <<std::endl;
        pos_object.x = x;
    	pos_object.y = y;
   		press_buttom = 1;
    }

}

void Camera::ShapeDetect()
{ 
	
	cv::Mat src_gray;
	cv::Mat dst ;

    /* Convert it to rgb*/
  	cvtColor( scene, dst, CV_GRAY2RGB );
	
  	/* Convert to binary image using Canny*/
	cv::Mat bw;
	cv::Canny(scene, bw, 0, 200, 5);

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
			BottonCHosen.Botton_2frame = BottonCHosen.Center_;
			setLabel(dst, "BOTP", CenterAndContours.second[info_geometry.first]);
			BottonCHosen.Bot_C = CenterAndContours.second[info_geometry.first];

			std::pair<int,int> value = FindMaxValue(dst, BottonCHosen.Center_ );
			cv::Mat roi(scene, cv::Rect(BottonCHosen.Center_.x - 30,BottonCHosen.Center_.y - 40,value.first, value.second));
			cv::Mat convert_BcMat_ = roi.clone(); 

			convert_BcMat_.convertTo(BottonCHosen.figure_, CV_8U) ;

		    /* SIFT feature detector and feature extractor */
		    cv::SiftFeatureDetector detector( 0.01, 3.0 );
		    cv::SiftDescriptorExtractor extractor( 2.0 );

		    detector.detect(BottonCHosen.figure_, BottonCHosen.keyp_ );
		    extractor.compute( BottonCHosen.figure_, BottonCHosen.keyp_, BottonCHosen.descr_ );

			cv::imshow("dst", dst);
			cv::waitKey(0);

			// cv::imshow("dst", roi);
			// cv::waitKey(0);
	
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
	
	tf::poseMsgToKDL ((msg->pose).pose, frame_so3_ptam);
	FillCamMatrixPose(frame_so3_ptam);
}



void Camera::FillCamMatrixPose(KDL::Frame frame)
{

	Camera2_S03.at<float>(0,0) = frame_so3_ptam.M.data[0];
	Camera2_S03.at<float>(0,1) = frame_so3_ptam.M.data[1];
	Camera2_S03.at<float>(0,2) = frame_so3_ptam.M.data[2];
	
	Camera2_S03.at<float>(0,3) = frame_so3_ptam.p.x();
	Camera2_S03.at<float>(1,0) = frame_so3_ptam.M.data[3];
	Camera2_S03.at<float>(1,1) = frame_so3_ptam.M.data[4];
	Camera2_S03.at<float>(1,2) = frame_so3_ptam.M.data[5];

	Camera2_S03.at<float>(1,3) = frame_so3_ptam.p.y();
	Camera2_S03.at<float>(2,0) = frame_so3_ptam.M.data[6];
	Camera2_S03.at<float>(2,1) = frame_so3_ptam.M.data[7];
	Camera2_S03.at<float>(2,2) = frame_so3_ptam.M.data[8];

	Camera2_S03.at<float>(2,3) = frame_so3_ptam.p.z();
}




void Camera::DetectWithSift()
{
	ROS_INFO_STREAM("DENTRO DetectWithSift");
	cv::Mat frame;
	cv::Mat frame_temp;
	frame_temp = frame1_.clone();
	frame_temp.convertTo(frame, CV_8U) ;
  	
  	//Detect sift
	std::vector<cv::KeyPoint> keyp_;
	cv::Mat descr_;   		
	cv::SiftFeatureDetector detector( 0.01, 3.0 );
	cv::SiftDescriptorExtractor extractor( 2.0 );

	detector.detect(frame, keyp_ );
	extractor.compute( frame, keyp_, descr_ );
 		
    /* -- Step 2: Matching descriptor vectors using FLANN matcher */
 	cv::FlannBasedMatcher matcher;
 	std::vector< cv::DMatch > matches;
    matcher.match( BottonCHosen.descr_, descr_, matches );
   
    double max_dist = 0; double min_dist = 90;

	/*-- Quick calculation of max and min distances between keypoints */
	for( int i = 0; i < BottonCHosen.descr_.rows; i++ )
	{ 
	 	double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	}

	std::vector< cv::DMatch > good_matches;

	for( int i = 0; i < BottonCHosen.descr_.rows; i++ )
	{ 
	 	if( matches[i].distance <= cv::max(2*min_dist, 0.02) )
	    { 
		   	good_matches.push_back( matches[i]);

		   	KeypointIm2.push_back((keyp_[matches[i].trainIdx]).pt); 
	    }
    }

	/*-- Draw only "good" matches */
	cv::Mat img_matches;
	cv::drawMatches(BottonCHosen.figure_, BottonCHosen.keyp_, frame, keyp_, good_matches, img_matches, 
	  				cv::Scalar::all(-1), cv::Scalar::all(-1),std::vector<char>(), 
	  				cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	// cv::imshow("good matches", img_matches);
	// cv::waitKey(0);


	/*-- Localize the object */
	std::vector<cv::Point> scene_point;
	ROS_INFO_STREAM(" good_matches.size(): " << good_matches.size());
	for(unsigned int i = 0; i < good_matches.size(); i++ )
	{
	    //-- Get the keypoints from the good matches
	    KeyPointIm1Match.push_back( BottonCHosen.keyp_[ good_matches[i].queryIdx ].pt );
	    scene_point.push_back( keyp_[ good_matches[i].trainIdx ].pt );
	}

	// 	//-- Show detected matches
	if(scene_point.size() >0)
	{
		setLabel(frame, "quip", scene_point);
		cv::Rect r = cv::boundingRect(scene_point);
		cv::Point pt(r.x + (r.width / 2), r.y + (r.height / 2));
		cv::line( frame,pt, pt ,  cv::Scalar( 5, 5, 0 ),  2, 8 );
		BottonCHosen.Botton_2frame = pt;
		cv::imshow("Object detection",frame);
		cv::waitKey(0);
	}
}


void Camera::Triangulation()
{
	ROS_INFO("*******Triangulation*******");

	if(InfoKf3d_ == true)
	{	
		std::vector<cv::Point3d> vect3d;
		vect3d = ConvertPointFromWordToCam();
		ProjectPointAndFindPosBot3d(vect3d);
	}
	else
		ROS_INFO_STREAM("***Ti stai dimenticando di inviare ptam visualizer ***");
		
	stop_flag = false;
}


std::vector<cv::Point3d> Camera::ConvertPointFromWordToCam()
{
	std::vector<cv::Point3d> vect3d_return;
    Eigen::MatrixXd	So3_ptam_eigen(4,4); 
    FromMatToEigen( Camera2_S03, So3_ptam_eigen );

    std::ofstream myfile2, myfile3;
    myfile2.open("/home/daniela/code/src/eye_in_hand/filelog_camera.txt");
    myfile3.open("/home/daniela/code/src/eye_in_hand/filelog_word.txt");

  	for(unsigned int i=0; i < Ptamkf3d.size(); i++)
  	{
  		Eigen::VectorXd vect_eigen(4);
  		cv::Point3d point_temp(Ptamkf3d[i].x,Ptamkf3d[i].y,Ptamkf3d[i].z);
  		FromCvPointToEigen(point_temp, vect_eigen);
  		Eigen::VectorXd POint_c1_eigen(4);

  		Eigen::MatrixXd ScalinMatrix(4,4);
  		ScalinMatrix = Eigen::MatrixXd::Identity(4,4)*scala;
  		
  		POint_c1_eigen = ScalinMatrix*So3_ptam_eigen*vect_eigen;	//C_c_w*p_w
  		
  		FromEigenVectorToCvPOint(POint_c1_eigen, point_temp);
  		
    	vect3d_return.push_back(point_temp);

	 	myfile2<<POint_c1_eigen[0] << "\t" << POint_c1_eigen[1] << "\t" << POint_c1_eigen[2]<<"\n";
		myfile3 <<Ptamkf3d[i].x <<"\t"<<Ptamkf3d[i].y<< "\t" << Ptamkf3d[i].z<<"\n";
  	}

  	myfile2.close();
  	myfile3.close();

  	return vect3d_return;
}


void Camera::ProjectPointAndFindPosBot3d(std::vector<cv::Point3d> vect3d)
{
	std::vector<cv::Point3d> point_near;
	std::vector<double> min_vect;
	cv::Mat pc;
	frame1_.copyTo(pc);

	cv::Point2d point_temp(BottonCHosen.Botton_2frame.x, BottonCHosen.Botton_2frame.y);

	for(unsigned int i=0; i < vect3d.size(); i++)
	{
		cv::Point2d point_;
		point_.x =  cam_fx*vect3d[i].x/vect3d[i].z + cam_cx;
		point_.y =  cam_fy*vect3d[i].y/vect3d[i].z + cam_cy;
		// return_vect.push_back(point_);
		cv::line( pc,point_, point_ , cv::Scalar( 220, 220, 0 ),  2, 8 );	
		if(norm( (point_temp - point_)) <= 100)
		{
			point_near.push_back(vect3d[i]);
			min_vect.push_back(norm( (point_temp - point_)));
			cv::line( pc,point_, point_ , cv::Scalar( 5, 5, 0 ),  2, 8 );
		}
	}

	if(min_vect.size() >= 3 )	//servono almeno 3 punti per creare il piano
	{
		Eigen::Vector4f plane_param;
		plane_param = EstimatePlane(point_near);

		FindBottonPos3D(plane_param);
	}

	else
		ROS_INFO_STREAM("non ho trovato punti vicini. non posso calcolare il piano");
}