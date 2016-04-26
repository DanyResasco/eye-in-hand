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

	for (unsigned int i = 0; i < contours.size(); i++)
	{
	    // Approximate contour with accuracy proportional
	    // to the contour perimeter
	   cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.025, true);

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

	return CenterAndContours;
}


std::pair<int,int> FindMaxValue(cv::Mat &matrix, cv::Point &point )
{
	std::pair<int,int> dist;
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
		distance.push_back(local_dist);	
	}

	int min_d = distance[0];
	int count = 0;

	for(unsigned int i=0; i < distance.size(); i++)
	{
	   	if((min_d >= distance[i]) && (distance[i] < 90))
	   	{
			min_d = distance[i];
	   		check_bot.first = i;
	   		check_bot.second = true;
	   		count ++;
	   	}	
	}

	
	if(count == 0)
	{
	 	std::cout<<"Non è stato premuto correttamente il pulsante"<<std::endl;
	 	std::cout<<"Premere nuovamente il pulsante"<<std::endl;
		check_bot.second = false;
		check_bot.first = 1;

	}	
	
	return check_bot;
}

cv::Point FindACenter(std::vector<cv::Point> &geometry)
{
    cv::Moments m = moments(geometry, true);
    cv::Point center(m.m10/m.m00, m.m01/m.m00);

   return center;
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
