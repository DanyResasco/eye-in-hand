// using namespace std;
#include <camera_hand.h>

using namespace cv;
RNG rng(12345);

int main(int argc, char** argv)
{

	if (CV_MAJOR_VERSION == 2)
std::cout<<"Errore! Sto usando opencv 2!"<<std::endl;
if (CV_MAJOR_VERSION == 3)
std::cout<<"Ok sto usando opencv 3!"<<std::endl;
	Camera camera_local;
	
	char key;

	key = cv::waitKey(30);

	if (key == 27)                 // break if `esc' key was pressed. 
	{
		std::cout<<"hai premuto esc chiudo il programma"<<std::cout;
		return 0;
	}
	
	bool check;

	std::cout<<"ciao dany"<<std::endl;
	camera_local.ControllCamera();
	 	

	return 0;
}



void Camera::ControllCamera()
{
	// input data from calibration code

	// FileStorage fs;
 //    fs.open("out_camera_data.xml", FileStorage::READ);
 //    FileNode n = fs.root();
 //    for (FileNodeIterator current = n.begin(); current != n.end(); current++)
 //    {
 //        FileNode item = *current;
 //        //Mat v;
 //        item["Camera_Matrix"] >> Camera_Matrix;
 //        item["Distortion_Coefficients"] >> Distortion_Coefficients;
 //        // std::cout << Camera_Matrix <<std::endl;
 //    } 

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
		//if user has press the buttom
		if(start == 1)
		{
			cv::Mat scene2 = scene.clone();
			GetDisparityMap(scene2);
			DetectAndMove(scene2);
		}

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
	std::pair<int, bool> info_geometry;

	info_geometry = FindAMinDistanceButton(Center_Shape);

	if(info_geometry.second == true)
	{
		std::cout<<"** Tasto premuto correttamente **"<<std::endl;
		first_Step = 0;
		press_buttom = 0;
		BottonCHosen.Center_.x = floor(Center_Shape[info_geometry.first].x);
		BottonCHosen.Center_.y = floor(Center_Shape[info_geometry.first].y);
		//std::cout<<"hai premuto: "<<Center_.x<<  Center_.y <<std::endl;
		setLabel(dst, "BOTP", Shape_local[info_geometry.first]);
		BottonCHosen.Bot_C = Shape_local[info_geometry.first];
		
		// Sift
		//il valore 30 è stato dato per centrare meglio il bottone selezionato lasciando in vista
		//altri punti di riferimento
		cv::Mat roi(scene, Rect(BottonCHosen.Center_.x - 30,BottonCHosen.Center_.y - 40,100, 100));
		BottonCHosen.figure_= roi.clone();
		cv::imshow("CAMERA_ROBOT", roi);
		cv::Mat convert_BcMat_;
		scene.convertTo(convert_BcMat_, CV_8U) ;
		//Detect sift
		cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
		f2d->detect(convert_BcMat_, BottonCHosen.keyp_ );
		
		f2d->compute( convert_BcMat_, BottonCHosen.keyp_, BottonCHosen.descr_ );

		// cv::imshow("dst", dst);
		cv::waitKey(0);

		start = 1;
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
	// check_bot.first = 1;
	int count = 0;

	for(int i=0; i < distance.size(); i++)
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
	std::cout<<"check_bot.first: "<<check_bot.first<<std::endl;

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
	cv::Mat frame_cv;
	frame.convertTo(frame_cv, CV_8U); 
	//-- Step 1: Sift Detector
	cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create(); 
	//-- Step 1.1: Detect the keypoints:
	std::vector<KeyPoint> keypoints_2;    
	f2d->detect( frame_cv, keypoints_2 );
	//-- Step 1.2: Calculate descriptors (feature vectors)    
	cv::Mat descriptors_2;    
	f2d->compute( frame_cv, keypoints_2, descriptors_2 );
	std::cout<<"finito"<<std::endl;


 	//-- Step 3: Matching descriptor vectors using FLANN matcher
 	FlannBasedMatcher matcher;
 	std::vector< DMatch > matches;
    matcher.match( BottonCHosen.descr_, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

	// //-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < BottonCHosen.descr_.rows; i++ )
	{ 
	 	double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	}

	//   std::cout<<"-- Max dist : %f \n" << max_dist<<std::endl;
	//   std::cout<<"-- Min dist : %f \n" << min_dist<<std::endl;

	//   //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//   //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//   //-- small)
	//   //-- PS.- radiusMatch can also be used here.
	std::vector< DMatch > good_matches;

	for( int i = 0; i < BottonCHosen.descr_.rows; i++ )
	{ 
	 	if( matches[i].distance <= max(2*min_dist, 0.02) )
	    { 
		   	good_matches.push_back( matches[i]); 
	    }
    }

	//   //-- Draw only "good" matches
	Mat img_matches;
	drawMatches( BottonCHosen.figure_, BottonCHosen.keyp_, frame_cv, keypoints_2, good_matches, img_matches, 
	  				Scalar::all(-1), Scalar::all(-1),std::vector<char>(), 
	  				DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	//-- Show detected matches
	imshow( "Good Matches", img_matches );

	for( int i = 0; i < (int)good_matches.size(); i++ )
	{ 
	  	
	  	std::cout<< "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n" << i<<"\t"<< good_matches[i].queryIdx <<"\t"<< good_matches[i].trainIdx<<std::endl;

	}

	waitKey(0);
}

void Camera::GetDisparityMap(cv::Mat &frame_cv)
{
	std::cout<<"disparity"<<std::endl;
	int ndisparities = 16*5;  // < Range of disparity 
  	int SADWindowSize = 21; //< Size of the block window. Must be odd 
	cv::Ptr<StereoBM> stereo = cv::StereoBM::create(ndisparities, SADWindowSize);
 //    disparity = stereo.compute(imgL,imgR)
	cv::Mat OR_scene;
	cv::Mat new_frame;
    /// Convert it to gray
  	cvtColor( scene, OR_scene, CV_BGR2GRAY );
  	cvtColor( frame_cv, new_frame, CV_BGR2GRAY );

	Mat imgDisparity16S = Mat( scene.rows, scene.cols, CV_16S );
	imgDisparity8U = Mat( scene.rows, scene.cols, CV_8U );

	if( OR_scene.empty() || new_frame.empty() )
	{ 	
	 	std::cout<< " --(!) Error reading images " << std::endl; 
	 	exit; 
	}


	Ptr<StereoBM> sbm = StereoBM::create( ndisparities, SADWindowSize );

	//  -- 3. Calculate the disparity image
	sbm->compute( OR_scene, new_frame, imgDisparity8U );
	Depth = imgDisparity8U.at<double>(BottonCHosen.Center_.y,BottonCHosen.Center_.x);
	std::cout<<"Depth: "<<Depth<<std::endl;

	  //-- Check its extreme values
	// double minVal; double maxVal;

	// minMaxLoc( imgDisparity16S, &minVal, &maxVal );

	  // printf("Min disp: %f Max value: %f \n", minVal, maxVal);

	//  -- 4. Display it as a CV_8UC1 image
	// imgDisparity16S.convertTo( imgDisparity8U, CV_8U, 255/(maxVal - minVal));

	
	  // namedWindow( "windowDisparity", CV_WINDOW_AUTOSIZE );
	  // cv::imshow( "windowDisparity", imgDisparity8U );
	  // cv::waitKey(0);
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

