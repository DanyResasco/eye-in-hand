// using namespace std;
#include <camera_hand.h>

using namespace cv;
RNG rng(12345);

int main(int argc, char** argv)
{
	Camera camera_local;
	
	char key;

	key = cv::waitKey(30);

	if (key == 27)                 // break if `esc' key was pressed. 
	{
		std::cout<<"hai premuto esc chiudo il programma"<<std::cout;
		return 0;
	}
	
	bool check;

	camera_local.ControllCamera();
	
	
 	std::cout<<"ciao dany"<<std::endl;


	return 0;
}



void Camera::ControllCamera()
{
	// input data from calibration code
	FileStorage fs;
    fs.open("out_camera_data.xml", FileStorage::READ);
    FileNode n = fs.root();
    for (FileNodeIterator current = n.begin(); current != n.end(); current++)
    {
        FileNode item = *current;
        //Mat v;
        item["Camera_Matrix"] >> Camera_Matrix;
        item["Distortion_Coefficients"] >> Distortion_Coefficients;
        // std::cout << Camera_Matrix <<std::endl;
    } 

	// cv::VideoCapture cam(0); //open the camera
	char key;
	key = cv::waitKey(0);
	//create a window
	namedWindow("CAMERA_ROBOT", CV_WINDOW_AUTOSIZE);

	bool CamORfile = false ; //= cam.isOpened();
	 	
 	while(key != 27)
 	{
		if (CamORfile == true)
	 	{
	 		//camera is open
				// Mat frame_;
				// cam.read(frame_);
	 			// 	frame_width = cam.get(CV_CAP_PROP_FRAME_WIDTH);
				// 	frame_height = cam.get(CV_CAP_PROP_FRAME_HEIGHT);

				// imshow("CAMERA_ROBOT",frame_);
	 	}
	 	else
	 	{
	 		//std::cout<<"ciao dany non ho la cam accesa"<<std::endl;
       		//	std::cout<<"Unable to read stream from specified device"<<std::endl;
       		scene = imread("/home/daniela/Desktop/pollini/bott1.jpg", CV_LOAD_IMAGE_UNCHANGED);
		    if(!scene.data ) // Check for invalid input
		    {
		        std::cout<<"Could not open or find the image"<<std::endl; 
		    }

		 	imshow("CAMERA_ROBOT", scene);
	 	}

	 	// while(first_Step != 0)
	 	if (first_Step == 1)
	 	{
	 		//set the callback function for any mouse event
			setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);
			//std::cout<<"preso bott"<<std::endl;
			if(press_buttom == 1 )	//wait the mouse event
			{
				// std::cout<<"prima di shape"<<std::endl;
				ShapeDetect();
			}
		}
		Mat scene2 = scene.clone();
		DetectAndMove(scene2);
	 	if( waitKey (30) >= 0) break;
	}
}


void Camera::ShapeDetect()
{ 
	// std::vector<geometry> Shape_;


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

	// Shape_.resize(contours.size());
	// The array for storing the approximation curve
	std::vector<cv::Point> approx;

	// We'll put the labels in this destination image
	cv::Mat dst = scene.clone();
	// std::vecto<int> = Index_Shape;
	std::vector<cv::Point>  Center_Shape;
	std::vector<std::vector<cv::Point> > Shape_local;;

	//std::cout<<"trovati i contorni"<<std::endl;
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
	            // Detect geometry.Shape_contours or square
	            cv::Rect r = cv::boundingRect(contours[i]);
	            double ratio = std::abs(1 - (double)r.width / r.height);

	            setLabel(dst, ratio <= 0.02 ? "SQU" : "RECT", contours[i]);
	            
	            Center_Shape.push_back(FindACenter(contours[i]));
	            Shape_local.push_back(contours[i]);
	            
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
	            Center_Shape.push_back(FindACenter(contours[i]));
	            Shape_local.push_back(contours[i]);
	                  
	        }
	    }
	}

	// std::cout<<"center size: "<<Center_Shape.size()<<std::endl;
	// std::cout<<"contours size: "<<Shape_local.size()<<std::endl;

	std::pair<int, bool> info_geometry;

	info_geometry = FindAMinDistanceButton(Center_Shape);

	if(info_geometry.second == true)
	{
	
		first_Step = 0;
		press_buttom = 0;
		CorretObjectPos.x = floor(Center_Shape[info_geometry.first].x);
		CorretObjectPos.y = floor(Center_Shape[info_geometry.first].y);
		//std::cout<<"hai premuto: "<<CorretObjectPos.x<<  CorretObjectPos.y <<std::endl;
		setLabel(dst, "BOTP", Shape_local[info_geometry.first]);
		BottonCHosen = Shape_local[info_geometry.first];
		cv::imshow("dst", dst);
		cv::waitKey(0);
	}
	else
	{
		std::cout<<"riprova"<<std::endl;
	}
		
}


std::pair<int, bool> Camera::FindAMinDistanceButton(std::vector<cv::Point> &baricentro)
{
	int  local_dist;
	std::vector<int> distance;
	std::pair<int, bool> check_bot;
	check_bot.second = false;
	
	for(int i=0; i<= baricentro.size();i++)
	{
		local_dist = norm((pos_object - baricentro[i]));
		// std::cout<<"distance 14: "<<local_dist<<std::endl;

		distance.push_back(local_dist);	
	}
	

	int min_d = distance[0];
	check_bot.first = 0;

	for(int i=0; i < distance.size(); i++)
	{
	   	if((min_d >= distance[i]) && (distance[i] < 90))
	   	{
			min_d = distance[i];
	   		check_bot.first = i;
	   		check_bot.second = true;
	   		//std::cout<<"index: "<<index_shape <<std::endl;


	   	}
					  	
	  	else
	 	{
	 		// std::cout<<"Non è stato premuto correttamente il pulsante"<<std::endl;
	 		// std::cout<<"Premere nuovamente il pulsante"<<std::endl;
			continue;
	  	}
	}


	

	return check_bot;

}











cv::Point FindACenter(std::vector<cv::Point> &geometry)
{
    cv::Moments m = moments(geometry, true);
    cv::Point center(m.m10/m.m00, m.m01/m.m00);

   return center;
}




void Camera::DetectAndMove(cv::Mat &frame)
{
	//-- Step 1: Detect the keypoints  using Sift Detector
	// cv::SiftFeatureDetector detector;
 //   	// detector(1, 1, cv::SIFT::CommonParams::DEFAULT_NOCTAVES,cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
	// 	      // cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, cv::SIFT::CommonParams::FIRST_ANGLE );
   
 //    std::vector<cv::KeyPoint> keypoints1;	//one is the original scene
 //    detector.detect(scene, keypoints1);
 //    // Add results to image and save.
 //    cv::Mat output1;
 //    cv::drawKeypoints(scene, keypoints1, output1);
 //    cv::imshow("Sift_result1.jpg", output1);
 //    cv::imwrite("Sift_result1.jpg",output1);
  
 //    //keypoints array for input 2
 //    std::vector<cv::KeyPoint> keypoints2;
 //    detector.detect(frame,keypoints2);
 //    //output array for ouput 2
 //    cv::Mat output2;
 //    cv::drawKeypoints(frame,keypoints2,output2);
 //    cv::imshow("Sift_result2.jpg",output2);
 //    cv::imwrite("Sift_result2.jpg",output2);
   
 //    //-- Step 2: Detect the descriptors  using Sift extractor
 //    cv::SiftDescriptorExtractor extractor;
 //    cv::Mat descriptors1,descriptors2;
 //    extractor.compute(scene,keypoints1,descriptors1);
 //    extractor.compute(frame,keypoints2,descriptors2);

 //    //-- Step 3: Matching descriptor vectors using FLANN matcher
 //    FlannBasedMatcher matcher;
 //    std::vector< DMatch > matches;
 //    matcher.match( descriptors1, descriptors2, matches );

 //    double max_dist = 0; double min_dist = 100;

	// //-- Quick calculation of max and min distances between keypoints
	// for( int i = 0; i < descriptors1.rows; i++ )
	// { 
	// 	double dist = matches[i].distance;
	//     if( dist < min_dist ) min_dist = dist;
	//     if( dist > max_dist ) max_dist = dist;
	// }

	//   std::cout<<"-- Max dist : %f \n" << max_dist<<std::endl;
	//   std::cout<<"-- Min dist : %f \n" << min_dist<<std::endl;

	//   //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//   //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//   //-- small)
	//   //-- PS.- radiusMatch can also be used here.
	//   std::vector< DMatch > good_matches;

	//   for( int i = 0; i < descriptors1.rows; i++ )
	//   { 
	//   	if( matches[i].distance <= max(2*min_dist, 0.02) )
	//     { 
	//     	good_matches.push_back( matches[i]); 
	//     }
	//   }

	//   //-- Draw only "good" matches
	//   Mat img_matches;
	//   drawMatches( scene, keypoints1, frame, keypoints2, good_matches, img_matches, 
	//   				Scalar::all(-1), Scalar::all(-1),std::vector<char>(), 
	//   				DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	//   //-- Show detected matches
	//   imshow( "Good Matches", img_matches );

	//   for( int i = 0; i < (int)good_matches.size(); i++ )
	//   { 
	  	
	//   	std::cout<< "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n" << i<<"\t"<< good_matches[i].queryIdx <<"\t"<< good_matches[i].trainIdx<<std::endl;

	//   }

	//   waitKey(0);
	 // SIFT sift;
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



}



	



void Camera::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if   ( event == EVENT_LBUTTONDOWN )
    {
       std::cout << "Left mouse button is clicked. Save the position (" << x << ", " << y << ")" <<std::endl;
        pos_object.x = x;
    	pos_object.y = y;
   		press_buttom = 1;
    }

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

