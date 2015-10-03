#include <camera_hand.h>

using namespace cv;

int main(int argc, char** argv)
{
	//ros::init(argc, argv, "Robot_hand_scene");
	Camera camera_local;
	
	char key;

	key = cv::waitKey(30);

	if (key == 27)                 // break if `esc' key was pressed. 
	{
		std::cout<<"hai premuto esc chiudo il programma"<<std::cout;
		return 0;
	}
	// //ros::NodeHandle nodeH;
	bool check;

	
	// std::string nome_finestra ("CAMERA_ROBOT");
	check = camera_local.ControllCamera();
	
 	std::cout<<"ciao dany"<<std::endl;

	// while (ros::ok())
	// {

	// 	//loop_rate.sleep();
	// 	ros::spinOnce();

	// }


	return 0;
}



bool Camera::ControllCamera()
{
	// cv::VideoCapture cam(0); //open the camera
	char key;
	key = cv::waitKey(0);
	//create a window
	namedWindow("CAMERA_ROBOT", CV_WINDOW_AUTOSIZE);

	bool CamORfile = false ; //= cam.isOpened();
	int first_Step = 1;
 	
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
       		scene = imread("/home/daniela/Desktop/pollini/bott.jpg", CV_LOAD_IMAGE_UNCHANGED);
		    if(!scene.data ) // Check for invalid input
		    {
		        std::cout<<"Could not open or find the image"<<std::endl; 
		    }

		 	imshow("CAMERA_ROBOT", scene);
	 	}

	 	if(first_Step == 1)
	 	{
	 		//set the callback function for any mouse event
			setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);
			if(press_buttom ==1 )	//wait the mouse event
			{
				ShapeDetect();
				first_Step = 0;
			}
		}
		
	 	if( waitKey (30) >= 0) break;
		
		
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
	// char k = cv::waitKey(30);
	// if(k !=10)
	// {	
	// 	setMouseCallback("CAMERA_ROBOT", CallBackFunc, NULL);
	// }
	return CamORfile;

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
	// // std::vector<Point2f>center(contours.size()  );
	// // std::vector<Moments> mu(contours.size() );
	// std::vector<std::vector<cv::Point> > circle;
	// // std::vector<float>radius_cerchio( contours.size() );
  	int num_bott_cerchio = 0;
  	std::vector<std::vector<cv::Point> > circle;

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

	        	//GetCenter(contours[i]);
	            setLabel(dst, "CIR", contours[i]);
	            num_bott_cerchio ++;
	            circle.push_back(contours[i]);
	            std::cout<<"numero di cerchi "<<num_bott_cerchio<<std::endl;
	            std::cout<<"circle.size(): "<<circle.size()<<std::endl;
	        }
	    }
	}
	//FindACenter()
	std::vector<Point2f>center(circle.size()  );
	std::vector<Moments> mu(circle.size() );
	std::vector<float>radius_cerchio( circle.size() );
	std::vector<int> distance;
	std::vector<Point2f> mc( circle.size() );

	if(num_bott_cerchio > 0)
	{
		for(int i=0; i<circle.size();i++)
		{	// Get the moments
		   	mu[i] = moments( circle[i], false );
		   	// std::cout<<"mu: "<<mu[i].m10 <<mu[i].m00 <<mu[i].m01 <<std::endl;
		   	mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
		   	// std::cout<<"mc: "<<mc[i]<<std::endl;
		   	int  local_dist;
	 		cv::Point baricentro;
	 		baricentro.x = floor(mc[i].x);
			baricentro.y = floor(mc[i].y);
			// std::cout<<"center i x: "<<baricentro.x <<std::endl;
			// std::cout<<"center i y: "<<baricentro.y <<std::endl;

			local_dist = norm((pos_object - baricentro));
			distance.push_back(local_dist);
		}
				
		//std::cout<<"distance.size: "<<distance.size()<<std::endl; 

		int min_d = distance[0];
		int index_circle;
		for(int i=0; i< distance.size(); i++)
		{
		   	if(min_d > distance[i])
		   	{
				min_d = distance[i];
		   		index_circle = i;
		   		//std::cout<<"index circle: "<<index_circle<<std::endl;
		   	}
					  	
		  	else
		 	{
				continue;
		  	}

		}

		CorretObjectPos.x = floor(mc[index_circle].x);
		CorretObjectPos.y = floor(mc[index_circle].y);
		std::cout<<"hai premuto: "<<CorretObjectPos.x<<  CorretObjectPos.y <<std::endl;
		setLabel(dst, "BOTP", circle[index_circle]);
	}
	else
	{
		std::cout<<"nessun bottone trovato"<<std::endl;
	}
	
	//cv::imshow("src", scene);
	cv::imshow("dst", dst);
	cv::waitKey(0);

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






void Camera::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if   ( event == EVENT_LBUTTONDOWN )
    {
       std::cout << "Left mouse button is clicked. Save the position (" << x << ", " << y << ")" <<std::endl;
        pos_object.x = x;
    	pos_object.y = y;
   		press_buttom = 1;
    }

   

    //cv2.rectangle(img,(384,0),(510,128),(0,255,0),3)
}


// void Camera::DuplicateScene()
// {



// }