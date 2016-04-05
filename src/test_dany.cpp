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
				// ROS_INFO_STREAM("entrata qui");
				// imshow("CAMERA_ROBOT_MOVE",  node.scene);
				node.DetectWithSift();

				// if((node.sub_ptam1 == true) && (node.sub_ptam == true))
				// {
				// 	node.Find3dPoint();
				// }



				// waitKey(0);
				// if(node.FirstCalibration == 1)
				// {
				// 	node.StereoCalibration();
				// }
				// else
				// {
					// ROS_INFO_STREAM("Dentro else");
					// ROS_INFO_STREAM("node.sub_ptam: " <<std::boolalpha<<node.sub_ptam);
					// ROS_INFO_STREAM("node.sub_ptam2: " <<std::boolalpha<<node.sub_ptam_2);
					// if((node.sub_ptam == true) && (node.sub_ptam_2 == true))
					// {
				 //   		// node.Triangulation();
				 //   		node.MatchPtamCv();
					// }
				// }
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
	// float cam_fx, cam_d0, cam_d1, cam_d2, cam_d3,cam_d4, cam_fy, cam_cx, cam_cy;
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
	// nh.param<double>("/Camera_demo/scale_factor", scale_factor,2.0);
	nh.param<double>("/Camera_demo/distanzaWebcam", distanzaWebcam,2.0);

	// ROS_INFO_STREAM("distanzaWebcam: " <<distanzaWebcam);

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

 	// ROS_INFO_STREAM("CAM ne lcostruttore: " << Camera_Matrix);

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
	// So3_new = 0;
	sub_ptam_2 = false;
	SaveFirst = false;
	scala_ok = false;
	sub_ptam1 = false;



	sub = it_.subscribe("/camera/output_video", 1, &Camera::ImageConverter, this);
	ptam_sub = nh.subscribe("/vslam/pose",1, &Camera::SOtreCamera, this);  //word in camera frame
	srv_move = nh.subscribe("/moveok",1, &Camera::MoveCallBack, this);
	movewebcamrobot = nh.subscribe("/moverobot",1, &Camera::RobotMove,this);
	ptam_kf3d = nh.subscribe("/vslam/pc2",1,&Camera::InfoKf3d,this);
	// dany_kf = nh.subscribe("/dany_kf",1,&Camera::InfoKf2d,this);

	// dany_kf = nh.subscribe("/dany_kf2d",1,&Camera::InfoKf2d,this);
	// dany_kf3d = nh.subscribe("/dany_kf3d",1,&Camera::InfoKf3d,this);
	// ptam_kf3d = nh.subscribe("/vslam/kfs",1,&Camera::InfoKf3d,this);
	// ptam_kf3d = nh.subscribe("/dany_pose",1,&Camera::POSE3d,this);
	// marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_dany", 1);
}


// void Camera::POSE3d(const geometry_msgs::Pose::ConstPtr msg)
// {
// 	KDL::Frame frame_pose_temp;
// 	tf::poseMsgToKDL(*msg, frame_pose_temp);
// 	frame_vect.push_back(frame_pose_temp);
// }



// void Camera::Find3dPoint()
// {
// 	// sub_ptam1 = false;
// 	// sub_ptam = false;
// 	ROS_INFO_STREAM("QUI");
// 	ROS_INFO_STREAM("vect3d.SIZE()"<<vect3d.size() );
// 	ROS_INFO_STREAM("point_near.SIZE()"<<point_near.size() );
// 	// std::vector<cv::Point3d> p3d;

// 	// visualization_msgs::Marker points;
// 	visualization_msgs::MarkerArray points_vect;


// 	// points.header.frame_id =  "world";
//  //    points.header.stamp =  ros::Time::now();
//  //    points.ns =  "points_kf";
//  //    points.action = visualization_msgs::Marker::ADD;
//  //    points.pose.orientation.w =  1.0;
//  //    points.type = visualization_msgs::Marker::POINTS;
//  //     // POINTS markers use x and y scale for width/height respectively
//  //    points.scale.x = 0.2;
//  //    points.scale.y = 0.2;
//  //    // Points are green
//  //    points.color.g = 1.0f;
//  //    points.color.a = 1.0;
//  // 	points.lifetime = ros::Duration(100);




// 	for(unsigned int i=0; i < point_near.size();i++)
// 	{	
// 		visualization_msgs::Marker points;
// 		geometry_msgs::Point p;
// 		// p3d.push_back(vect3d[index_near[i]]);
// 		// ROS_INFO_STREAM("vect3d[index_near[i]: " <<vect3d[index_near[i]]);
// 		p.x = vect3d[index_near[i]].x;
// 		p.y = vect3d[index_near[i]].y;
// 		p.z = vect3d[index_near[i]].z;
// 		points.points.push_back(p);
// 		points.header.frame_id =  "world";
// 	    points.header.stamp =  ros::Time::now();
// 	    points.ns =  "points_kf";
// 	    points.action = visualization_msgs::Marker::ADD;
// 	    points.pose.orientation.w =  1.0;
// 	    points.type = visualization_msgs::Marker::POINTS;
// 	     // POINTS markers use x and y scale for width/height respectively
// 	    points.scale.x = 0.2;
// 	    points.scale.y = 0.2;
// 	    // Points are green
// 	    points.color.g = 1.0f;
// 	    points.color.a = 1.0;
// 	 	points.lifetime = ros::Duration(100);
// 	 	points_vect.markers.push_back(points);
// 	}


// 	// points.header.frame_id =  "world";
//  //    points.header.stamp =  ros::Time::now();
//  //    points.ns =  "points_kf";
//  //    points.action = visualization_msgs::Marker::ADD;
//  //    points.pose.orientation.w =  1.0;
//  //    points.type = visualization_msgs::Marker::POINTS;
//  //     // POINTS markers use x and y scale for width/height respectively
//  //    points.scale.x = 0.2;
//  //    points.scale.y = 0.2;
//  //    // Points are green
//  //    points.color.g = 1.0f;
//  //    points.color.a = 1.0;
//  // 	points.lifetime = ros::Duration(1000);

//     marker_pub.publish(points_vect);
//     //   tf::Transform tfGeomTRansform;
//     // tf::poseMsgToTF( p, tfGeomTRansform );
//     //  tf::TransformBroadcaster transform_ptam.sendTransform( tf::StampedTransform( tfGeomTRansform, ros::Time::now(), "world", "test") );
// }



void Camera::InfoKf2d(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	int index_kf = 0;
	kf_point.clear();
	cv::Mat pc;
	scene.copyTo(pc);
	for(unsigned int i=0; i < msg->data.size();i++)
	{
		cv::Point2d temp_kf(msg->data[index_kf],msg->data[index_kf+1] );
		kf_point.push_back(temp_kf);
		index_kf = index_kf +2;
		line( pc,temp_kf, temp_kf , Scalar( 110, 220, 0 ),  2, 8 );
	}

	
	imshow("kf",pc);
	waitKey(0);
	MatchPtamCv(kf_point);
}


// void Camera::InfoKf3d(const std_msgs::Float32MultiArray::ConstPtr &msg)
// {
// 	int index_kf = 0;
// 	kf_point3d.clear();
// 	// cv::Mat pc;
// 	// scene.copyTo(pc);
// 	for(unsigned int i=0; i < msg->data.size();i++)
// 	{
// 		cv::Point3d temp_kf(msg->data[index_kf],msg->data[index_kf+1],msg->data[index_kf+2] );
// 		kf_point3d.push_back(temp_kf);
// 		ROS_INFO_STREAM("temp_kf: " << temp_kf);
// 		index_kf = index_kf +3;
// 		// line( pc,temp_kf, temp_kf , Scalar( 110, 220, 0 ),  2, 8 );
// 	}

	
// 	// imshow("kf",pc);
// 	// waitKey(0);
// 	// MatchPtamCv(kf_point);
// }














void FromMatToEigen(cv::Mat Mat_, Eigen::MatrixXd &Eigen)
{

	for( int i=0; i < Mat_.rows; i++)
	{
		for( int j =0; j < Mat_.cols -1 ; j++)
		{
			Eigen(i,j) = Mat_.at<double>(i,j);
		}
	}

	// ROS_INFO_STREAM("qui");
	Eigen(3,0) = 0;
	Eigen(3,1) = 0;
	Eigen(3,2) = 0;
	Eigen(3,3) = 1;

	// ROS_INFO_STREAM("finito la matrice eigen");
}

void FromCvPointToEigen(cv::Point3d point_, Eigen::VectorXd &vect)
{
	vect[0] = point_.x;
	vect[1] = point_.y;
	vect[2] = point_.z;
	vect[3] = 1;
}


void FromEigenVectorToCvPOint(Eigen::VectorXd Eigen, cv::Point3d &mat)
{
	mat.x = Eigen[0];
	mat.y = Eigen[1];
	mat.z = Eigen[2];
}

// void  ConvertToHomogeneousEigen(cv::Mat So3_ptam_r, Eigen::MatrixXd &So3_ptam_eigen)
// {

// }


// void Camera::InfoKf3d(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
// 		ROS_INFO_STREAM("qui InfoKf3d");
// 	pcl::PointCloud<pcl::PointXYZ> Ptamkf3d;
//   	// pcl::fromROSMsg (*msg, Ptamkf3d);
// 	pcl::PCLPointCloud2 pcl_pc;
//     pcl_conversions::toPCL(*msg, pcl_pc);
//     pcl::fromPCLPointCloud2(pcl_pc, Ptamkf3d);  	// std::vector<cv::Point3d> vect3d;
//   	vect3d.clear();
//   	// projectedPoints.clear();

//   	// ROS_INFO_STREAM("Ptamkf3d.size(): "<< Ptamkf3d.size());

//   	cv::Mat So3_ptam_r;
//     Camera2_S03.convertTo(So3_ptam_r, CV_64F);

//     Eigen::MatrixXd	So3_ptam_eigen(4,4); 
//     FromMatToEigen(So3_ptam_r,So3_ptam_eigen );
//     std::ofstream myfile;
//     myfile.open("/home/daniela/code/src/eye_in_hand/filelog.txt");

//   	for(unsigned int i=0; i < Ptamkf3d.size(); i++)
//   	{
//   		// ROS_INFO_STREAM("Ptamkf3d: " << Ptamkf3d[i]);
//   		// ROS_INFO_STREAM("Ptamkf3d: " << Ptamkf3d[i].x);
//   		Eigen::VectorXd vect_eigen(4);
//   		cv::Point3d point_temp(Ptamkf3d[i].x,Ptamkf3d[i].y,Ptamkf3d[i].z);
//   		// ROS_INFO_STREAM("Ptamkf3d[i].x: " << Ptamkf3d[i].x);
//   		// ROS_INFO_STREAM("Ptamkf3d[i].y: " << Ptamkf3d[i].y);
//   		// ROS_INFO_STREAM("Ptamkf3d[i].z: " << Ptamkf3d[i].z);
//   		FromCvPointToEigen(point_temp, vect_eigen);
//   		Eigen::VectorXd POint_c1_eigen(4);
//   		POint_c1_eigen = So3_ptam_eigen*vect_eigen;	//C_c_w*p_w
  		
//   		FromEigenVectorToCvPOint(POint_c1_eigen, point_temp);
//   		ROS_INFO_STREAM("point_temp: " <<point_temp);
//     	vect3d.push_back(point_temp);
//     	ROS_INFO_STREAM("point_tempvect3d[i]: " <<vect3d[i]);

// 		if (myfile.is_open())
// 		{
// 		  	// myfile << "POint 3d x,y,z \n";
//   		  		myfile <<Ptamkf3d[i].x <<"\t"<<Ptamkf3d[i].y<< "\t" << Ptamkf3d[i].z<<"\n";
		    
// 		    // ROS_INFO("STO SALVANDO");
// 		}
// 		// else
// 		// 	ROS_INFO("bau");
//   	}
  	
//  //    // ROS_INFO_STREAM("vect3d: " << vect3d.size());
//  //    cv::Mat T(3,1,CV_64F); // translation vector
//  //    T.at<double>(0,0) = 0;
//  //    T.at<double>(1,0) = 0;
//  //    T.at<double>(2,0) = 0;

//  //    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);	//rotation matrix
 
//  //    cv::Mat rvecR(3,1,CV_64F);
// 	// cv::Rodrigues(R,rvecR);//rodrigues rotation matrix
	
//     // std::vector<cv::Point2d> projectedPoints;
// 	// cv::Mat Camera_Matrix_double;
//  //    Camera_Matrix.convertTo(Camera_Matrix_double, CV_64F);
//  //    cv::Mat distr;
//  //    Cam_par_distortion.convertTo(distr, CV_64F);

//  //    projectedPoints = ProjectPointWithPtamCamParam(vect3d);

    
//  //    // cv::projectPoints(vect3d, rvecR, T, Camera_Matrix_double, distr, projectedPoints);
//  //    ROS_INFO_STREAM("projectedPoints: " << projectedPoints.size());

//  //   	RNG rng(12345);
//  //   	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
   	
//  //   	for (size_t i=0; i<projectedPoints.size(); i++)
//  //   	{
//  //   		cv::Point2d temp_point(std::abs(projectedPoints[i].x),std::abs(projectedPoints[i].y) );
//  //   		ROS_INFO_STREAM("projectedPoints: " << projectedPoints[i]);
//  //   		ROS_INFO_STREAM("temp_point: " << temp_point);
// 	//     circle(scene, temp_point, 4, color,2,8,0);

// 	//  //    if (myfile.is_open())
// 	// 	// {
// 	// 	//   	// myfile << "POint 3d x,y,z \n";
//  //  	// 		myfile <<projectedPoints[i].x <<"\t"<<projectedPoints[i].y<< "\n";		    
// 	// 	//     // ROS_INFO("STO SALVANDO");
// 	// 	// }
// 	// 	// else
// 	// 	// 	ROS_INFO("bau");
// 	// }

// 	myfile.close();
//     // cv::imshow("kf_ptam",scene_first );
//     // cv::waitKey(0); 
//     sub_ptam = true; 
//     // MatchPtamCv(); 
//     Find3dPoint();
// }


// std::vector<cv::Point2d> Camera::ProjectPointWithPtamCamParam(std::vector<cv::Point3d> vect3d)
// {
// 	std::vector<cv::Point2d> return_vect;

// 	for(int i=0; i < vect3d.size(); i++)
// 	{
// 		cv::Point2d point_;
// 		point_.x =  cam_fx*vect3d[i].x/vect3d[i].z + cam_cx;
// 		point_.y =  cam_fy*vect3d[i].y/vect3d[i].z + cam_cy;
// 		return_vect.push_back(point_);
// 	}
	
// 	return return_vect;
// }


void Camera::MatchPtamCv(std::vector<cv::Point2d> vect)
{
	// ROS_INFO_STREAM("qui");
	// std::vector<cv::Point2d> point_near;
	// std::vector<cv::Point3d> point_3d;
	cv::Point2d point_temp(BottonCHosen.Center_.x, BottonCHosen.Center_.y);
	// std::vector<int> index_near;
	point_near.clear();
	index_near.clear();
	ROS_INFO_STREAM("vect.size(): "<<vect.size());
	// int count = 0;
	cv::Mat pc;
	scene.copyTo(pc);
	ROS_INFO_STREAM("point_temp: " << point_temp);
	for(int i=0; i < vect.size(); i++)
	{	
		ROS_INFO_STREAM("vect[i]: " <<vect[i]);
		cv::Point2d test_p((point_temp - vect[i]).x,(point_temp - vect[i]).y);
		ROS_INFO_STREAM("test_p: " << test_p);
		
		ROS_INFO_STREAM("test_p.norm: " << norm(test_p));
		if(norm( (point_temp - vect[i])) <= 100)
		{
			point_near.push_back(vect[i]);
			index_near.push_back(i);
			// ROS_INFO_STREAM("index_near: " << index_near[i]);
				line( pc,vect[i], vect[i] , Scalar( 220, 220, 0 ),  2, 8 );
			// point_3d.push_back(vect3d[i]);
			// ROS_INFO_STREAM("IL BOTTONE E' IN POSIZIONE: " <<point_3d); 
		}
	}


	

	
	imshow("kf_near",pc);
	waitKey(0);
	ROS_INFO_STREAM("point_near.size()"<< point_near.size());
	// ROS_INFO_STREAM("COUNT: " << count);
}



void Camera::RobotMove(const geometry_msgs::Pose msg)
{
	ROS_INFO_STREAM("RICEVUTO Messaggio");
	// geometry_msgs::Point position;
	// geometry_msgs::Quaternion 
	// KDL::Quaternion orientation(msg->orientation.x,msg->orientation.y,msg->orientation.z);
	tf::poseMsgToKDL(msg, Move_robot);
	// Move_robot.z = msg->position.z;
	// Move_robot.y = msg->position.y;
	// Move_robot.x = msg->position.x;
	// move_z_robot = msg->position.z;

	sub_ptam_2 = true;
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
	scene.copyTo(scene_first);
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
	// KDL::Frame frame_so3_ptam;
	// ROS_INFO("LEGGO TOPIC ptam");
	tf::poseMsgToKDL ((msg->pose).pose, frame_so3_ptam);
	// ROS_INFO_STREAM("frame_so3_ptam: "<<frame_so3_ptam);
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

		sub_ptam1 = true;
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

    double max_dist = 0; double min_dist = 90;

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



void Camera::MoveCallBack(const std_msgs::Bool::ConstPtr msg)
{

	move_camera_end = msg->data;
	frame1_ = scene.clone();
	if(frame1_.empty())
	{
		ROS_ERROR("empty mat frame");
		// return 0;
	}

}



void Camera::Triangulation()
{
	ROS_INFO("*******Triangulation*******");
	// double distance_z;

	if(scala_ok == false)
	{
		if(SaveFirst == true)
		{
			So3_prev_ptam = frame_so3_ptam;
			// S03_prev_robot = Move_robot;
			// distance_z = media_z;
			SaveFirst = false;
			// scala = media_z/So3_prev;
			// ROS_INFO_STREAM("Per Ptam invece sono in : " << So3_prev*scala_prev);
		}
		else
		{
			double temp_distance_z_ptam;
			temp_distance_z_ptam = So3_prev_ptam.p.z() - frame_so3_ptam.p.z();
			scala = Move_robot.p.z()/temp_distance_z_ptam;
			scala_ok = true;
		}
	}
	else
	{
		KDL::Frame New_web_pose;
		// for(unsigned int i=0; i<frame_so3_ptam.M.size();i++)
		// {
		// 	New_web_pose.M.data[i] = frame_so3_ptam.data[i]*scala;
		// }
		New_web_pose.M = frame_so3_ptam.M;
		New_web_pose.p = KDL::Vector::Zero();
		New_web_pose.p.data[0] = frame_so3_ptam.p.x()*scala;
		New_web_pose.p.data[1] = frame_so3_ptam.p.y()*scala;
		New_web_pose.p.data[2] = frame_so3_ptam.p.z()*scala;


		ROS_INFO_STREAM("New_web_pose: " << New_web_pose);
	}





	// KDL::Frame Tpunto;

	// KDL::Rotation rot_stereo = KDL::Rotation::Identity();
	// KDL::Vector trasl_stereo(distanzaWebcam,0.0,0.0);
	// KDL::Frame Tc1cs(rot_stereo,trasl_stereo );
	// KDL::Rotation Mat_scala = KDL::Rotation::Identity();
	// Mat_scala(0,0) = (1/scala);
	// Mat_scala(1,1) = (1/scala);
	// Mat_scala(2,2) = (1/scala);
	
	// KDL::Vector t(0,0,0);
	// KDL::Frame Ptam_temp(Mat_scala,t);
	// KDL::Frame Ptam_scalate = Ptam_temp*frame_so3_ptam;

	// Tpunto = Ptam_scalate*Tc1cs;

	// ROS_INFO_STREAM("Tpunto: " << Tpunto);

	// if(SaveFirst == true)
	// {
	// 	So3_prev = std::abs(Camera2_S03.at<float>(2,3));
	// 	distance_z = media_z;
	// 	SaveFirst = false;
	// 	ROS_INFO_STREAM("Ora sono in: " << distance_z);

	// }
	// else
	// {
		// if(So3_prev < std::abs(Camera2_S03.at<float>(2,3)))
		// {
		// 	ROS_INFO("PTAM da i numeri.. trova una soluzione");
		// }
		// else
		// {
			// scale = move_z_robot/std::abs(Camera2_S03.at<float>(2,3));

			// scale = move_z_robot/std::abs(Camera2_S03.at<float>(2,3));
		 	// So3_new = So3_new + scala_prev*So3_prev - std::abs(Camera2_S03.at<float>(2,3))*scale;
		 	// distance_z =  media_z - So3_new;
		 	 
		 	// ROS_INFO_STREAM("Ora sono in: " << distance_z);
		 	// ROS_INFO_STREAM("OPPURE SONO IN: " << scala_prev*So3_prev - std::abs(Camera2_S03.at<float>(2,3))*scale);
		 	// scala_prev = scale;
		 	// So3_prev = std::abs(Camera2_S03.at<float>(2,3)); 
		// }
	// }

	sub_ptam_2 = false;


}


