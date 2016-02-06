// using namespace std;
#include <test_whit_ptam_and_ros.h>

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
        if(node.arrived_cam ==1)
        {
            	
    		if(node.Finish == 1)
    		{
    			imshow("CAMERA_ROBOT",  node.scene);
    			// std::cout<<"qui"<<std::endl;
    			node.ControllCamera();
    			waitKey(0);
    		}

    		// bool move_camera_end = node.MoveCamera();
    		if(node.move_camera_end == true)
    		{
    			node.DetectWithSift(node.scene);
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
	ros::param::get("~video_or_photo", video_or_photo);
	ROS_DEBUG( "video_or_photo %lf", video_or_photo);

	nh.param<double>("/Camera_demo/threshold_sift",threshold_sift,0.01);
	std::cout<<"threshold_sift: "<<threshold_sift<<std::endl;
	
    image_pub_ = it_.advertise("/camera/output_video", 1);	
	sub = it_.subscribe("/usb_cam/image_raw", 1, &Camera::ImageConverter, this);
	move_camera_end = false;

}


void Camera::ImageConverter(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<"dany ho ricevuto la camera"<<std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv_ptr->image.copyTo(scene);

    sensor_msgs::ImagePtr msg2;
    msg2 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, scene).toImageMsg();
    	
    image_pub_.publish(msg2);

	arrived_cam = 1;

      //       imshow("CAMERA_ROBOT", scene);
    		
    		// ControllCamera();
    		// waitKey(0);
    	
}


void Camera::ControllCamera()
{
	std::cout<<"ControllCamera"<<std::endl;
	
	if (first_Step == 1)
	{
	 	//set the callback function for any mouse event
		setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);
		//std::cout<<"preso bott"<<std::endl;
		if(press_buttom == 1 )	//wait the mouse event
		{
			
			ShapeDetect();
			// first_Step = 0;
			// std::cout<<"muovi"<<std::endl;
		}
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
	// std::vector<geometry> Shape_;
	Mat src_gray;
	cv::Mat dst = scene.clone();
    /// Convert it to gray
  	cvtColor( scene, src_gray, CV_BGR2GRAY );

  	// Reduce the noise so we avoid false circle detection
  	GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
  	// Convert to binary image using Canny
	cv::Mat bw;
	cv::Canny(src_gray, bw, 0, 50, 5);
	std::pair<std::vector<cv::Point> ,std::vector<std::vector<cv::Point> >> CenterAndContours;
	CenterAndContours = FindContours(bw.clone(), dst.clone());
	
	// std::cout<<"Shape_local: "<<Shape_local.size()<<std::endl;;
	if(CenterAndContours.second.size() > 0)
	{
		std::pair<int,bool> info_geometry;
		cv::Point point_b;
		point_b.x = pos_object.x;
		point_b.y = pos_object.y;

		

		info_geometry = FindAMinDistanceButton(CenterAndContours.first,point_b);
		
		if(info_geometry.second == true)
		{
			std::cout<<"** Tasto premuto correttamente **"<<std::endl;
			first_Step = 0;
			press_buttom = 0;
			BottonCHosen.Center_.x = floor(CenterAndContours.first[info_geometry.first].x);
			BottonCHosen.Center_.y = floor(CenterAndContours.first[info_geometry.first].y);
			// std::cout<<"hai premuto: "<<BottonCHosen.Center_.x<< '\t'<< BottonCHosen.Center_.y <<std::endl;
			setLabel(dst, "BOTP", CenterAndContours.second[info_geometry.first]);
			BottonCHosen.Bot_C = CenterAndContours.second[info_geometry.first];

			std::pair<int,int> value = FindMaxValue(dst, BottonCHosen.Center_ );
			cv::Mat roi(scene, Rect(BottonCHosen.Center_.x - 30,BottonCHosen.Center_.y - 40,value.first, value.second));
			cv::Mat convert_BcMat_ = roi.clone(); 

			convert_BcMat_.convertTo(BottonCHosen.figure_, CV_8U) ;
			//Detect sift
			 /* threshold      = 0.04;
	       		edge_threshold = 10.0;
	       		magnification  = 3.0;    */ 
		    // SIFT feature detector and feature extractor
		    cv::SiftFeatureDetector detector( 0.01, 3.0 );
		    cv::SiftDescriptorExtractor extractor( 2.0 );

		    detector.detect(BottonCHosen.figure_, BottonCHosen.keyp_ );
		    extractor.compute( BottonCHosen.figure_, BottonCHosen.keyp_, BottonCHosen.descr_ );

			cv::imshow("dst", dst);
			cv::waitKey(0);

			// start = 1;
			Finish = 0;
			move_camera_end = true;
			first_Step = 0;
		}
		else
		{
			press_buttom = 0;
			std::cout<<"ripremi nuovamente il pulsante"<<std::endl;
		}

	}
	else
	{
		std::cout<<"ripremi nuovamente il pulsante"<<std::endl;
		press_buttom = 0;
	}		
}

std::pair<std::vector<cv::Point> ,std::vector<std::vector<cv::Point> >> Camera::FindContours(cv::Mat bw, cv::Mat camera)
{
	// Find contours
	std::pair<std::vector<cv::Point> ,std::vector<std::vector<cv::Point>> > CenterAndContours;
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// The array for storing the approximation curve
	std::vector<cv::Point> approx;

	// We'll put the labels in this destination image
	cv::Mat dst = camera.clone();

	//std::cout<<"trovati i contorni"<<std::endl;
	for (unsigned int i = 0; i < contours.size(); i++)
	{
	    // Approximate contour with accuracy proportional
	    // to the contour perimeter
	   cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
			continue;

	    if (approx.size() == 3)
        setLabel(dst, "TRI", contours[i]);    // Triangles

    	else if (approx.size() >= 4 && approx.size() <= 6)
    	{
	        // Number of vertices of polygonal curve
	        int vtc = approx.size();

	        // Get the degree (in cosines) of all corners
	        std::vector<double> cos;
	        for (int j = 2; j < vtc+1; j++)
	            cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

	        // Sort ascending the corner degree values
	        std::sort(cos.begin(), cos.end());

	        // Get the lowest and the highest degree
	        double mincos = cos.front();
	        double maxcos = cos.back();

	        // Use the degrees obtained above and the number of vertices
	        // to determine the shape of the contour
	        if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
	        {
	            // Detect geometry.Shape_contours or square
	            cv::Rect r = cv::boundingRect(contours[i]);
	            double ratio = std::abs(1 - (double)r.width / r.height);

	            setLabel(dst, ratio <= 0.02 ? "SQU" : "RECT", contours[i]);
	            
	            CenterAndContours.first.push_back(FindACenter(contours[i]));
	            CenterAndContours.second.push_back(contours[i]);
	            
	        }
	        else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
	             setLabel(dst, "PENTA", contours[i]);
	        else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
	            setLabel(dst, "HEXA", contours[i]);
    	}
    	else
	    {
	        // Detect and label circles
	        double area = cv::contourArea(contours[i]);
	        cv::Rect r = cv::boundingRect(contours[i]);
	        int radius = r.width / 2;

	        if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
	        {

	            setLabel(dst, "CIR", contours[i]);
	            CenterAndContours.first.push_back(FindACenter(contours[i]));

	            CenterAndContours.second.push_back(contours[i]);
	        }
	    }
	}
	// imshow("circle", dst);
	// waitKey(0);
	return CenterAndContours;
}












std::pair<int,int> FindMaxValue(cv::Mat &matrix, cv::Point &point )
{
	std::pair<int,int> dist;
	// std::cout<<"point x: " <<point.x<<'\t'<<"point y: "<<point.y<<std::endl;
	// std::cout<<"matrix x: " <<matrix.cols<<'\t'<<"matrix y: "<<matrix.rows<<std::endl;
	cv::Point point2(point.x-30, point.y -40);
	cv::Rect rect(point, matrix.size());
	
	if((point2.x)+100 < matrix.cols )
	{
		dist.first = 100;
	}
	else
		dist.first = matrix.cols - point2.x;

	if((point2.y)+100 < matrix.rows )
	{
		dist.second = 100;
	}
	else
		dist.second = matrix.rows - point2.y;

	// std::cout<<"dist first: "<<dist.first<<'\t'<<"dist second: "<<dist.second<<std::endl;
	return dist;
}






std::pair<int,bool> Camera::FindAMinDistanceButton(std::vector<cv::Point> &baricentro, cv::Point &point_)
{
	int  local_dist;
	std::vector<int> distance;
	std::pair<int, bool> check_bot;
	check_bot.second = false;
	
	for(unsigned int i=0; i<= baricentro.size();i++)
	{
		local_dist = norm((point_ - baricentro[i]));
		// std::cout<<"distance 14: "<<local_dist<<std::endl;

		distance.push_back(local_dist);	
	}

	int min_d = distance[0];
	// check_bot.first = 1;
	int count = 0;

	for(unsigned int i=0; i < distance.size(); i++)
	{
	   	if((min_d >= distance[i]) && (distance[i] < 90))
	   	{
			min_d = distance[i];
	   		check_bot.first = i;
	   		check_bot.second = true;
	   		count ++;
	   		//std::cout<<"index: "<<index_shape <<std::endl;
	   	}	
	}

	
	if(count == 0)
	{
	 	std::cout<<"Non è stato premuto correttamente il pulsante"<<std::endl;
	 	std::cout<<"Premere nuovamente il pulsante"<<std::endl;
		check_bot.second = false;
		check_bot.first = 1;

	}	
	// std::cout<<"check_bot.first: "<<check_bot.first<<std::endl;

	return check_bot;
	

}

cv::Point FindACenter(std::vector<cv::Point> &geometry)
{
    cv::Moments m = moments(geometry, true);
    cv::Point center(m.m10/m.m00, m.m01/m.m00);

   return center;
}


void Camera::DetectWithSift(cv::Mat &frame)
{
	//Detect sift
		 /* threshold      = 0.04;
       		edge_threshold = 10.0;
       		magnification  = 3.0;    */ 
	    // SIFT feature detector and feature extractor
    std::cout<<"BottonCHosen.descr_.rows: "<<BottonCHosen.descr_.rows<<std::endl;
    std::vector<KeyPoint> keyp_;
	cv::Mat descr_;   		
	cv::SiftFeatureDetector detector( 0.01, 3.0 );
	cv::SiftDescriptorExtractor extractor( 2.0 );
	// cv::SiftFeatureDetector detector;
	// cv::SiftDescriptorExtractor extractor;
	detector.detect(frame, keyp_ );
	extractor.compute( frame, keyp_, descr_ );
 	
 	// -- Step 2: Matching descriptor vectors using FLANN matcher
 	FlannBasedMatcher matcher;
 	std::vector< DMatch > matches;
    matcher.match( BottonCHosen.descr_, descr_, matches );

    double max_dist = 0; double min_dist = 70;

    std::cout<<"good_matches.size(): "<<matches.size()<<std::endl;
      // std::cout<<"BottonCHosen.descr_.rows: "<<BottonCHosen.descr_.rows<<std::endl;

	// //-- Quick calculation of max and min distances between keypoints
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

	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches( BottonCHosen.figure_, BottonCHosen.keyp_, frame, keyp_, good_matches, img_matches, 
	  				Scalar::all(-1), Scalar::all(-1),std::vector<char>(), 
	  				DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	//-- Localize the object
	std::vector<Point2f> obj;
	std::vector<Point> scene_point;

	for( int i = 0; i < good_matches.size(); i++ )
	{
	    //-- Get the keypoints from the good matches
	    obj.push_back( BottonCHosen.keyp_[ good_matches[i].queryIdx ].pt );
	    scene_point.push_back( keyp_[ good_matches[i].trainIdx ].pt );
	}
	if(scene_point.size() >0)
	{
		setLabel(frame, "quip", scene_point);
		imshow("Object detection",frame);
		waitKey(0);
	}
	
	//-- Show detected matches
	// imshow( "Good Matches", img_matches );
	// waitKey(0);
}





void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
