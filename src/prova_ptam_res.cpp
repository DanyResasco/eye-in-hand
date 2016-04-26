// using namespace std;
#include <ptam_scale.h>
#include <Scale_estimation.hpp>

// using namespace cv;
cv::RNG rng(12345);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "eye_in_hand_node");
	PtamScale node;
	
	ROS_INFO("[eye_in_hand] Node is ready");

	double spin_rate = 50;
	ros::param::get("~spin_rate",spin_rate);
	
	ROS_DEBUG( "Spin Rate %lf", spin_rate);

	cv::namedWindow("PtamScale_ROBOT");

	ros::Rate rate(spin_rate); 
		
	while ( ros::ok() )
    {
           
        //relax to fit output rate
        rate.sleep(); 
        ros::spinOnce();            
    }

	return 0;
}

PtamScale::PtamScale()
{
	
 	count_n_passi = 0;	
	scala = 1;
	// myfile1.open("/home/daniela/code/src/eye_in_hand/pos_log.txt");
	// myfile.open("/home/daniela/code/src/eye_in_hand/scale_log.txt");
	myfile4.open("/home/daniela/code/src/eye_in_hand/ptam_pose7.txt");

	// sub = it_.subscribe("/PtamScale/output_video", 1, &PtamScale::ImageConverter, this);
	ptam_sub = nh.subscribe("/vslam/pose",1, &PtamScale::SOtreCamera, this);  //word in camera framebu
	movewebcamrobot = nh.subscribe("/moverobot",1, &PtamScale::RobotMove,this); // robot in cam frame
	stop_sub = nh.subscribe("/stop",1,&PtamScale::StopCallback,this);	//to stop the pc2 callback
	// ptam_kf3d = nh.subscribe("/vslam/pc2",1,&PtamScale::InfoKf3d,this);	//point in word frame
	pub_scala = nh.advertise<std_msgs::Float32>("/scala_", 1);
}

void PtamScale::StopCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stop_flag = msg->data;
	
}

void PtamScale::RobotMove(const geometry_msgs::Pose msg)
{
	ROS_INFO_STREAM("RICEVUTO Messaggio");
	tf::poseMsgToKDL(msg, Move_robot);
	// Robot.push_back(Move_robot.p.z());
	So3_prev_ptam = frame_so3_ptam;
}




void PtamScale::SOtreCamera(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
	 
		tf::poseMsgToKDL ((msg->pose).pose, frame_so3_ptam);
		// FillCamMatrixPose(frame_so3_ptam);
		frame_w_c = frame_so3_ptam.Inverse();  

		if(stop_flag == false)
    	{ 

    		KDL::Frame Frame_c2_c1;
			Frame_c2_c1 = So3_prev_ptam*frame_w_c;

			if(myfile4.is_open())
			{
				myfile4 << Move_robot.p.z()<<'\t';
				myfile4 << Frame_c2_c1.p.z() << '\n';
			}
			
			else
				ROS_INFO_STREAM("no myfile4");

			Robot.push_back(Move_robot.p.z());
    		Ptam.push_back(Frame_c2_c1.p.z());
    		scala = Scale(Ptam, Robot);
			ROS_INFO_STREAM("scala: " << scala);
			Vect_scala.push_back(scala);
			// So3_prev_ptam = frame_w_c;
			
			if( Vect_scala.size() > 250 )
			{	
				if((*Vect_scala.end() - Vect_scala[Vect_scala.size() -1]) <= 0.001  )
				{
					std_msgs::Float32 scala_;
					scala_.data = Vect_scala.back();
					ROS_INFO_STREAM("scala_.data"<<scala_.data);
					pub_scala.publish(scala_);
					stop_flag = true;
					double scala_mio = ScalaReturn(Frame_c2_c1.p.z(), So3_prev_ptam.p.z(), Move_robot.p.z());
					ROS_INFO_STREAM("scala_mio: " << scala_mio);
				}
			}
		}
}


