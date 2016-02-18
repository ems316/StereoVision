#include "opencv2/opencv.hpp"
using namespace cv;

int main(int, char**)
{
  VideoCapture cap(0);
		     //"udpsrc port=5003 ! application/x-rtp ! rtpjitterbuffer //latenc=0 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, edges, CV_RGB2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

