#include <camera_hand.h>

using namespace cv;

int main(int argc, char** argv)
{
	//ros::init(argc, argv, "Robot_hand_scene");
	char k;

	//k=cvWaitKey(0);

	// //ros::NodeHandle nodeH;
	Camera camera_local;

	camera_local.ControllCamera();
	// // //set the callback function for any mouse event
 // 	camera_local.setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);

	camera_local.ShapeDetect();
	// //setMouseCallback("CAMERA_ROBOT", camera_local.CallBackFunc, NULL);

	// while(k != 'ESC')
	// {
		Mat frame_;
		// cam.read(frame_);
  //   	camera_local.DuplicateScene(frame_);
  //   }
		std::cout<<"ciao dany"<<std::endl;

	// while (ros::ok())
	// {

	// 	//loop_rate.sleep();
	// 	ros::spinOnce();

	// }


	return 0;
}



void Camera::ControllCamera()
{
	// cv::VideoCapture cam(0); //open the camera

	//create a window
	namedWindow("CAMERA_ROBOT", CV_WINDOW_AUTOSIZE);

	// if (!cam.isOpened())  //if not success read the the file
	// {
		std::cout<<"ciao dany non ho la cam accesa"<<std::endl;
       //	std::cout<<"Unable to read stream from specified device"<<std::endl;
    	//Mat image;
   		// Mat img = imread("MyPic.JPG", CV_LOAD_IMAGE_UNCHANGED);;   // Read the file
   		// VideoCapture cap("NOME_DEL_FILE_CHE_DEVE_LEGGERE.avi"); // open the video file for reading
       	scene = imread("/home/daniela/Desktop/pollini/bott.jpg", CV_LOAD_IMAGE_UNCHANGED);
	    if(!scene.data ) // Check for invalid input
	    {
	        std::cout<<"Could not open or find the image"<<std::endl;
	       
	    }	
	    // imshow("CAMERA_ROBOT", cap);
	    // imshow("CAMERA_ROBOT", img);
	    // imshow("CAMERA_ROBOT", scene);
	    // waitKey(0); //wait infinite time for a keypress

	// }
	
	// else
	// {
	// 	std::cout<<"ciao dany ho la cam accesa e luchino fa il birichino"<<std::endl;
	// 	frame_width = cam.get(CV_CAP_PROP_FRAME_WIDTH);
	// 	frame_height = cam.get(CV_CAP_PROP_FRAME_HEIGHT);


	// 	while (1)
 //    	{
	//         Mat frame;

	//         bool bSuccess = cam.read(frame); // read a new frame from video

	//          if (!bSuccess) //if not success, break loop
	//         {
	//              std::cout << "Cannot read a frame from video stream" <<std::endl;
	//              break;
	//         }

	//         imshow("CAMERA_ROBOT", frame);
	//     }
	// 	// cam.read(scene);
	// 	// sift = cv2.SIFT();
	// 	// kp1, des1 = sift.detectAndCompute(scene,None);

	// 	imshow("CAMERA_ROBOT", scene);

	// 	//if esc key is pressed break loop
	// 	// if(waitKey(30) == 27)
	// 	// {
	// 	// 	break;
	// 	// }
	
	// }
//posso farlo solo se ho fatto la calibrazione
// 	cv::Matx31f hom_pt(point_in_image.x, point_in_image.y, 1);
// hom_pt = camera_intrinsics_mat.inv()*hom_pt; //put in world coordinates

// cv::Point3f origin(0,0,0);
// cv::Point3f direction(hom_pt(0),hom_pt(1),hom_pt(2));
	//set the callback function for any mouse event
   // setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);

	// Setup a rectangle to define your region of interest
	// Rect myROI(Data_point(0), Data_point(1), 100, 100);

	// Crop the full image to that image contained by the rectangle myROI
	// Note that this doesn't copy the data
// 	Mat croppedImage = image(myROI);



}

void Camera::ShapeDetect()
{ 
	Mat src_gray;
    /// Convert it to gray
  	cvtColor( scene, src_gray, CV_BGR2GRAY );

  	// Reduce the noise so we avoid false circle detection
  	GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
  	// Convert to binary image using Canny
	Mat bw;
	cv::Canny(src_gray, bw, 0, 50, 5);

	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// The array for storing the approximation curve
	std::vector<cv::Point> approx;

	// We'll put the labels in this destination image
	cv::Mat dst = scene.clone();
	std::vector<Point2f>center( contours.size() );
  	std::vector<float>radius( contours.size() );
  	int num_bott_cerchio = 0;

	for (int i = 0; i < contours.size(); i++)
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
	            // Detect rectangle or square
	            cv::Rect r = cv::boundingRect(contours[i]);
	            double ratio = std::abs(1 - (double)r.width / r.height);

	            setLabel(dst, ratio <= 0.02 ? "SQU" : "RECT", contours[i]);
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
	        	minEnclosingCircle( (Mat)contours[i], center[i], radius[i] );
	        	//GetCenter(contours[i]);
	            setLabel(dst, "CIR", contours[i]);
	            num_bott_cerchio ++;
	        }
	    }
	} // end of for() loop
 //    std::vector<Vec3f> circles;

 //    if(src_gray.empty())
 //    {
 //     std::cout<<"src_gray è vuota"<<std::endl;
	// }

  // /// Apply the Hough Transform to find the circles
 //  	HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 90, 80, 0, 0 );
 //  	std::cout<<"dany cerco i bottoni"<<std::endl;
	std::vector<double> distance;
	distance.resize(circles.size());
	// std::cout<<"numero di cerchi "<<circles.size()<<std::endl;
  // /// Draw the circles detected
	for( size_t i = 0; i < num_bott_cerchio.size(); i++ )
	{	
	// 	// std::cout<<"dany sono dentro al for"<<std::endl;
	//   	  Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	//       int radius = cvRound(circles[i][2]);
	//       // circle center
	//       circle( scene, center, 3, Scalar(0,255,0), -1, 8, 0 );
	//       // circle outline
	//       circle( scene, center, radius, Scalar(0,0,255), 3, 8, 0 );

	    //distance between circle[i] and select point
	    double local_dist;
	    local_dist = norm((pos_object - center[i]));
	    distance.push_back(local_dist);
	}

	  double min_d = distance[0];
	  int index_circle;
	  for(int i=0; i< distance.size(); i++)
	  {
	  	if(min_d > distance[i])
	  	{
	  		min_d = distance[i];
	  		index_circle = i;
	  	}
	  	
	  	else
	  	{
	  		continue;
	  	}

	  }

  // CorretObjectPos.x = cvRound(circles[index_circle][0]);
  // CorretObjectPos.y = cvRound(circles[index_circle][1]);



  // /// Show your results
  // namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  // imshow( "Hough Circle Transform Demo", scene );
	cv::imshow("src", scene);
	cv::imshow("dst", dst);
	cv::waitKey(0);


  
}

// void Camera::GetCenter(std::vector<cv::Point> objects)
// {
// 	cv::Point center;
// 	Eigen::Matrix3d bx = Eigen::Matrix3d::Zero();
// 	Eigen::Matrix3d by= Eigen::Matrix3d::Zero();;
// 	Eigen::Matrix3d a= Eigen::Matrix3d::Zero();;

// 	double A,B,C;
// 	A = pow(objects[0].x,2) +pow(objects[0].y,2);
// 	B = pow(objects[1].x,2) +pow(objects[1].y,2);
// 	C = pow(objects[2].x,2) +pow(objects[2].y,2); 
// 	//std::cout<<"dany ho calcolato a,b,c"<<std::endl;
	
// 	Eigen::Vector3d Row_bx1(A, objects[0].y, 1 );
// 	Eigen::Vector3d Row_bx2(B, objects[1].y, 1 );
// 	Eigen::Vector3d Row_bx3(C, objects[2].y, 1 );
	
// 	bx.row(0) << -Row_bx1.transpose();
// 	bx.row(1) << -Row_bx2.transpose();
// 	bx.row(2) << -Row_bx3.transpose();

// 	//std::cout<<"dany ho calcolato b"<<std::endl;

// 	Eigen::Vector3d Row_by1(A, objects[0].x, 1 );
// 	Eigen::Vector3d Row_by2(B, objects[1].x, 1 );
// 	Eigen::Vector3d Row_by3(C, objects[2].x, 1 );
	
// 	by.row(0) << Row_by1.transpose();
// 	by.row(1) << Row_by2.transpose();
// 	by.row(2) << Row_by3.transpose();

// 	//std::cout<<"dany ho calcolato by"<<std::endl;

// 	Eigen::Vector3d Row_a1(objects[0].x, objects[0].y, 1 );
// 	Eigen::Vector3d Row_a2(objects[1].x, objects[1].y, 1 );
// 	Eigen::Vector3d Row_a3(objects[2].x, objects[2].y, 1 );

// 	a.row(0) << -Row_a1.transpose();
// 	a.row(1) << -Row_a2.transpose();
// 	a.row(2) << -Row_a3.transpose();

// 	/std::cout<<"dany ho calcolato a"<<std::endl;

// 	//CorretObjectPos.x = 
// }

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
// void Camera::DuplicateScene(Mat &frame_t)
// {
// 	std::vetor<KeyPoint> kp2;
// 	Mat des2;
	
// 	//Initiate SIFT detector
// 	sift = cv2.SIFT()

// 	// find the keypoints and descriptors with SIFT
// 	//kp1, des1 = sift.detectAndCompute(img1,None)
// 	kp2, des2 = sift.detectAndCompute(img2,None)

// 	// BFMatcher with default params
// 	bf = cv2.BFMatcher()
// 	matches = bf.knnMatch(des1,des2, k=2)


// 	Mat scene_duplicate;
// 	scene_duplicate = scene;
	





	



// }





















// void Camera::CallBackFunc(int event, int x, int y, int flags, void* userdata)
// {
//     if ( flags == (EVENT_FLAG_CTRLKEY + EVENT_FLAG_LBUTTON) )
//     {
//        std::cout << "Left mouse button is clicked while pressing CTRL key - position (" << x << ", " << y << ")" <<std::endl;
//     }

//     pos_object.x = x;
//     pos_object.y = y;
    
// }


// void Camera::DuplicateScene()
// {



// }