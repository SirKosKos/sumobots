#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
//#include <cv.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

using namespace cv;
using namespace std;

int main() {

  Mat frame;
  VideoCapture cap(0);
  int k = 0;

  while (k < 50) {
    
    k++;
    // wait for a new frame from camera and store it into 'frame'
    cap.read(frame);
    // check if we succeeded
    if (frame.empty()) {
      cerr << "ERROR! blank frame grabbed\n";
      break;
    }
  }
    unsigned char *myData = frame.data;
    int cam_width = frame.cols;
    printf("cam_width = %d\n", cam_width);
    int cam_height = frame.rows;
    printf("cam_height = %d\n", cam_height);
    int widthStep = frame.step; //in case cols != strides
    int nChannels = frame.channels();


    // Setup a rectangle to define your region of interest
    Rect myROI(160, 0, 320, 120); // start drawing a rect from top-left 
                                  // corner with coords (x,y)=(160,0)
                                  
    // Numbering of x axis starts from right 
    // to left and of y-axis from top to bottom

    // Crop the full image to that image contained by the rectangle myROI
    // Note that this doesn't copy the data
    Mat croppedRef(frame, myROI);

    Mat cropped, cropped_gray, img_bw;
    // Copy the data into new matrix
    croppedRef.copyTo(cropped);

    cvtColor(cropped,cropped_gray,CV_RGB2GRAY);

    double thres_val = threshold(cropped_gray, img_bw, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    printf("Otcu threshold = %lf\n", thres_val);

    imshow("otsu", img_bw);
    imshow("greyscale", cropped_gray);
    imshow("cropped", cropped);
    imshow("original", frame);

    waitKey(0);
   
  
  return 0;
}

