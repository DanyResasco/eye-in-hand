#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char *argv[])
{
    int value = 50;
    int value2 = 0;

    // cvNamedWindow("main1",CV_WINDOW_NORMAL);
    cvNamedWindow("main2",CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);

    // cvCreateTrackbar( "track1", "main1", &value, 255,  NULL);//OK tested
    char* nameb1 = "button1";
    char* nameb2 = "button2";
    // cvCreateButton(nameb1,callbackButton,nameb1,CV_CHECKBOX,1);

    // cvCreateButton(nameb2,callbackButton,nameb2,CV_CHECKBOX,0);
    // cvCreateTrackbar( "track2", NULL, &value2, 255, NULL);
    // cvCreateButton("button5",callbackButton1,NULL,CV_RADIOBOX,0);
    // cvCreateButton("button6",callbackButton2,NULL,CV_RADIOBOX,1);

    // cvSetMouseCallback( "main2",on_mouse,NULL );

    // IplImage* img1 = cvLoadImage("files/flower.jpg");
    // IplImage* img2 = cvCreateImage(cvGetSize(img1),8,3);
    // cv::VideoCapture cam(0);
    // cv::Mat frame;
    CvCapture* video = cvCaptureFromCAM(0);
    IplImage* img3 = cvCreateImage(cvGetSize(cvQueryFrame(video)),8,3);

    while(cvWaitKey(33) != 27)
    {
        // cvAddS(img1,cvScalarAll(value),img2);
        cvAddS(cvQueryFrame(video),cvScalarAll(value2),img3);
        // cvShowImage("main1",img2);
        cvShowImage("main2",img3);
    }

    cvDestroyAllWindows();
    // cvReleaseImage(&img1);
    // cvReleaseImage(&img2);
    cvReleaseImage(&img3);
    cvReleaseCapture(&video);
    return 0;
}