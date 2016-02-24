#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher_for_ptam");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/output_video", 1);

  /*Open the webcam*/
  cv::VideoCapture cap(1);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);
  while (nh.ok()) 
  {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) 
    {
      cv::Mat src_gray;
      cvtColor( frame, src_gray, CV_BGR2GRAY );

      /* Reduce the noise*/
      GaussianBlur( src_gray, src_gray,cv::Size(9, 9), 2, 2 );

      /*send msgs to ptam*/
      msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", src_gray).toImageMsg();
      pub.publish(msg);
      
      cv::waitKey(1);
    }
    else
    {
      ROS_ERROR("No camera Found");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}