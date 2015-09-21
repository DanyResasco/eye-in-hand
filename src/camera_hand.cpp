#include <camera_hand.h>

using namespace cv;

int main(int argc, char** argv)
{
	//ros::init(argc, argv, "Robot_hand_scene");
	char k;

	k=cvWaitKey(0);

	//ros::NodeHandle nodeH;
	Camera camera_local;

	camera_local.ControllCamera();
	// //set the callback function for any mouse event
 	camera_local.setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);

	camera_local.GetButton();
	//setMouseCallback("CAMERA_ROBOT", camera_local.CallBackFunc, NULL);

	while(k != 'ESC')
	{
		Mat frame_;
		cam.read(frame_);
    	camera_local.DuplicateScene(frame_);
    }

	// while (ros::ok())
	// {

	// 	//loop_rate.sleep();
	// 	ros::spinOnce();

	// }


	return 0;
}



void Camera::ControllCamera()
{
	cv::VideoCapture cam(0); //open the camera

	//create a window
	namedWindow("CAMERA_ROBOT", CV_WINDOW_AUTOSIZE);

	if (!cam.isOpened())  //if not success read the the file
	{
       	std::cout<<"Unable to read stream from specified device"<<std::endl;
    	Mat image;
   		// Mat img = imread("MyPic.JPG", CV_LOAD_IMAGE_UNCHANGED);;   // Read the file
   		// VideoCapture cap("NOME_DEL_FILE_CHE_DEVE_LEGGERE.avi"); // open the video file for reading

	    if(!image.data ) // Check for invalid input
	    {
	        std::cout<<"Could not open or find the image"<<std::endl;
	       
	    }	
	    // imshow("CAMERA_ROBOT", cap);
	    // imshow("CAMERA_ROBOT", img);
	}
	
	else
	{
		frame_width = cam.get(CV_CAP_PROP_FRAME_WIDTH);
		frame_height = cam.get(CV_CAP_PROP_FRAME_HEIGHT);
		
		cam.read(scene);
		sift = cv2.SIFT();
		kp1, des1 = sift.detectAndCompute(scene,None);

		imshow("CAMERA_ROBOT", scene);

		//if esc key is pressed break loop
		// if(waitKey(30) == 27)
		// {
		// 	break;
		// }
	
	}
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

void Camera::GetButton()
{

  Mat src_gray;
  /// Convert it to gray
  cvtColor( scene, src_gray, CV_BGR2GRAY );

  /// Reduce the noise so we avoid false circle detection
  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

  std::vector<Vec3f> circles;

  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
  std::vector<double> distance;
  distance.resize(circles.size());

  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
  	  Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( scene, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( scene, center, radius, Scalar(0,0,255), 3, 8, 0 );

      //distance between circle[i] and select point
      double local_dist;
      local_dist = norm((pos_object- center));
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

  CorretObjectPos.x = cvRound(circles[index_circle][0]);
  CorretObjectPos.y = cvRound(circles[index_circle][1]);



  /// Show your results
  namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  imshow( "Hough Circle Transform Demo", scene );

  //waitKey(0);

  
}

void Camera::DuplicateScene(Mat &frame_t)
{
	std::vetor<KeyPoint> kp2;
	Mat des2;
	
	//Initiate SIFT detector
	sift = cv2.SIFT()

	// find the keypoints and descriptors with SIFT
	//kp1, des1 = sift.detectAndCompute(img1,None)
	kp2, des2 = sift.detectAndCompute(img2,None)

	// BFMatcher with default params
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2, k=2)


	Mat scene_duplicate;
	scene_duplicate = scene;
	





	



}





















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