// using namespace std;
#include <test_whit_ptam_and_ros.h>
// #include <function_camera.hpp>

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
   //      if(node.arrived_cam ==1)
   //      {            	
			// // if (node.first_Step == 1)
			// // {			    			
   // //  			// imshow("CAMERA_ROBOT",  node.scene);
   // //  			node.ControllCamera();
   // //  			// waitKey(0);
   // //  		}
   //  		// ROS_INFO_STREAM("ESCO DAL FIRST STEP");
			
   //  	}
            
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

		Camera2_S03 = cv::Mat(3,4,CV_32FC1,0.0f);



	sub = it_.subscribe("/camera/output_video", 1, &Camera::ImageConverter, this);
	ptam_sub = nh.subscribe("/vslam/pose",1, &Camera::SOtreCamera, this);  

	ptam_kf3d = nh.subscribe("/vslam/pc2",1,&Camera::InfoKf3d,this);
	// ptam_kf3d = nh.subscribe("vslam/kfs",1,&Camera::InfoKf3d,this);
}

void Camera::InfoKf3d(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	
	pcl::PointCloud<pcl::PointXYZ> Ptamkf3d;
  	pcl::fromROSMsg (*msg, Ptamkf3d);
  	// std::vector<cv::Point3d> vect3d;
  	vect3d.clear();
  	projectedPoints.clear();

  	// ROS_INFO_STREAM("Ptamkf3d.size(): "<< Ptamkf3d.size());

  	cv::Mat So3_ptam_r;
    Camera2_S03.convertTo(So3_ptam_r, CV_64F);

    Eigen::MatrixXd	So3_ptam_eigen(4,4); 
    FromMatToEigen(So3_ptam_r,So3_ptam_eigen );
    std::ofstream myfile;
    myfile.open("/home/daniela/code/src/eye_in_hand/filelog.txt");

  	for(unsigned int i=0; i < Ptamkf3d.size(); i++)
  	{
  		// ROS_INFO_STREAM("Ptamkf3d: " << Ptamkf3d[i]);
  		// ROS_INFO_STREAM("Ptamkf3d: " << Ptamkf3d[i].x);
  		Eigen::VectorXd vect_eigen(4);
  		cv::Point3d point_temp(Ptamkf3d[i].x,Ptamkf3d[i].y,Ptamkf3d[i].z);
  		// ROS_INFO_STREAM("Ptamkf3d[i].x: " << Ptamkf3d[i].x);
  		// ROS_INFO_STREAM("Ptamkf3d[i].y: " << Ptamkf3d[i].y);
  		// ROS_INFO_STREAM("Ptamkf3d[i].z: " << Ptamkf3d[i].z);
  		FromCvPointToEigen(point_temp, vect_eigen);
  		Eigen::VectorXd POint_c1_eigen(4);
  		POint_c1_eigen = So3_ptam_eigen.transpose()*vect_eigen;
  		
  		FromEigenVectorToCvPOint(POint_c1_eigen, point_temp);
  	
    	vect3d.push_back(point_temp);
    	// ROS_INFO_STREAM("vect3d: " << vect3d[i]);
    	// std::ofstream file;
    	// file.open("filelog.txt");
    	// file << "POint 3d x,y,z \n";
    	// file << Ptamkf3d[i].x <<"\t"<<Ptamkf3d[i].y << "\t" << Ptamkf3d[i].z;
    	// file.close();
    	 
    	
		if (myfile.is_open())
		{
		  	// myfile << "POint 3d x,y,z \n";
  		  		myfile <<Ptamkf3d[i].x <<"\t"<<Ptamkf3d[i].y<< "\t" << Ptamkf3d[i].z<<"\n";
		    
		    // ROS_INFO("STO SALVANDO");
		}
		else
			ROS_INFO("bau");
  	}
  	
    // ROS_INFO_STREAM("vect3d: " << vect3d.size());
 //    cv::Mat T(3,1,CV_64F); // translation vector
 //    T.at<double>(0,0) = 0;
 //    T.at<double>(1,0) = 0;
 //    T.at<double>(2,0) = 0;

 //    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);	//rotation matrix
 
 //    cv::Mat rvecR(3,1,CV_64F);
	// cv::Rodrigues(R,rvecR);//rodrigues rotation matrix
	
 //    // std::vector<cv::Point2d> projectedPoints;
	// cv::Mat Camera_Matrix_double;
 //    Camera_Matrix.convertTo(Camera_Matrix_double, CV_64F);
 //    cv::Mat distr;
 //    Cam_par_distortion.convertTo(distr, CV_64F);

    projectedPoints = ProjectPointWithPtamCamParam(vect3d);

    
    // cv::projectPoints(vect3d, rvecR, T, Camera_Matrix_double, distr, projectedPoints);
    ROS_INFO_STREAM("projectedPoints: " << projectedPoints.size());

   	RNG rng(12345);
   	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
   	
   	for (size_t i=0; i<projectedPoints.size(); i++)
   	{
   		ROS_INFO_STREAM("projectedPoints: " << projectedPoints[i]);
	    circle(scene, projectedPoints[i], 4, color,2,8,0);

	}

	myfile.close();
    cv::imshow("kf_ptam",scene_first );
    cv::waitKey(0); 
    sub_ptam = true; 
    // MatchPtamCv(); 
}


std::vector<cv::Point2d> Camera::ProjectPointWithPtamCamParam(std::vector<cv::Point3d> vect3d)
{
	std::vector<cv::Point2d> return_vect;

	for(int i=0; i < vect3d.size(); i++)
	{
		cv::Point2d point_;
		point_.x =  cam_fx*vect3d[i].x/vect3d[i].z + cam_cx;
		point_.y =  cam_fy*vect3d[i].y/vect3d[i].z + cam_cy;
		return_vect.push_back(point_);
	}
	
	return return_vect;
}



void Camera::ImageConverter(const sensor_msgs::Image::ConstPtr& msg)
{
	cv::Mat scene_temp;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv_ptr->image.copyTo(scene_temp);
    // cv::undistort(scene_temp, scene, Camera_Matrix, Cam_par_distortion);

	arrived_cam = 1;

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

	// sub_ptam = true;
}


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