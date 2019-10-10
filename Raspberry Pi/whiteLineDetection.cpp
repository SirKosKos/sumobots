#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#include <pigpio.h>
#include <pigpiod_if2.h>

using namespace cv;
using namespace std;

int in1 = 24, in2 = 23, en1 = 25, temp1 = 1;
int in3 = 27, in4 = 22, en2 = 17, temp2 = 1;
int high = 192, medium = 128, low = 90; //max = 255

int main(int argc,char* argv[]) {

  stringstream convert(argv[1]); // set up a stringstream variable named convert, initialized with the input from argv[1]
 
  int thres_val;
  if (!(convert >> thres_val)) // do the conversion
     thres_val = 0; // if conversion fails, set myint to a default value

  std::cout << "Got Otsu threshold: " << thres_val << '\n';

  int pi = gpioInitialise();

  if (pi < 0)
  {
    printf("pi = %d, pigpio initialisation failed.\n", pi); 
  }
  else
  {
    printf("pi = %d, pigpio initialised okay.\n", pi); 
  }

  gpioSetMode(in1, PI_OUTPUT); // Set GPIO24 as output. --> GPIO.setup(in1,GPIO.OUT)
  gpioSetMode(in2, PI_OUTPUT); // Set GPIO23 as output. --> GPIO.setup(in2,GPIO.OUT)
  gpioSetMode(en1, PI_OUTPUT);  // Set GPIO25 as output. --> GPIO.setup(en1,GPIO.OUT)
  set_PWM_frequency(pi, en1, 1000);

  gpioSetMode(in3, PI_OUTPUT); // Set GPIO27 as output. --> GPIO.setup(in3,GPIO.OUT)
  gpioSetMode(in4, PI_OUTPUT); // Set GPIO22 as output. --> GPIO.setup(in4,GPIO.OUT)
  gpioSetMode(en2, PI_OUTPUT);  // Set GPIO17 as output. --> GPIO.setup(en2,GPIO.OUT)
  set_PWM_frequency(pi, en2, 1000);

  gpioWrite(in1, 0); // Set GPIO24 low. --> GPIO.output(in1,GPIO.LOW)
  gpioWrite(in2, 0); // Set GPIO23 low. --> GPIO.output(in2,GPIO.LOW)

  gpioSetPWMfrequency(en1, 1000); // Set GPIO25 to 1000Hz. --> p=GPIO.PWM(en1,1000)

  gpioWrite(in3, 0); // Set GPIO27 low. --> GPIO.output(in3,GPIO.LOW)
  gpioWrite(in4, 0); // Set GPIO22 low. --> GPIO.output(in4,GPIO.LOW)

  gpioSetPWMfrequency(en2, 1000); // Set GPIO17 to 1000Hz. --> p=GPIO.PWM(en2,1000)

  gpioPWM(en1, low); // Sets GPIO25 on low speed --> p.start(25)
  gpioPWM(en2, low); // Sets GPIO25 on low speed --> p.start(25)

  Mat frame, frame_ref;
  //--- INITIALIZE VIDEOCAPTURE
  VideoCapture cap(0);
  cap.read(frame_ref);
  
  // Setup a rectangle to define your region of interest
  Rect myROI(160, 0, 320, 240);
  Mat croppedFrameRef(frame_ref, myROI);

  Mat cropped_ref, cropped_ref_gray;
  // Copy the data into new matrix
  croppedFrameRef.copyTo(cropped_ref);

  cvtColor(cropped_ref,cropped_ref_gray,CV_RGB2GRAY);

  unsigned char *cropRefData = cropped_ref.data;
  int crop_ref_width = cropped_ref.cols;
  int crop_ref_height = cropped_ref.rows;
  int widthStepCropRef = cropped_ref.step; //in case cols != strides
  int nChannelsCropRef = cropped_ref.channels();
  int numOfPixelsRef = crop_ref_width*crop_ref_height;
  int pixelValueRef[crop_ref_width*crop_ref_height], a=0, countRef = 0;
  double Mx;


  for(int i = 0; i < crop_ref_height; i++){
    for(int j = 0; j < crop_ref_width; j++){
      pixelValueRef[a] = (int)cropped_ref_gray.at<uchar>(i,j);
      a++;
    }
  }

  int n = sizeof(pixelValueRef)/sizeof(pixelValueRef[0]);
  sort(pixelValueRef, pixelValueRef+n);

  /*cout << "Array after sorting : \n"; 
  for (int i = 0; i < n; ++i) 
   cout << arr[i] << " ";*/

  for (int t=0; t < 300; t++) {
    countRef = countRef + pixelValueRef[t];
  }

  Mx = countRef/300;

   /* Main loop */
  while (1) {
        
    // wait for a new frame from camera and store it into 'frame'
    cap.read(frame);
    // check if we succeeded
    if (frame.empty()) {
      cerr << "ERROR! blank frame grabbed\n";
      break;
    }

    // show live and wait for a key with timeout long enough to show images
    /*imshow("Live", frame);
    if (waitKey(30) == 27)
      break;*/
    
    unsigned char *myData = frame.data;
    int cam_width = frame.cols;
    int cam_height = frame.rows;
    int numOfPixels = 320*240;
    int widthStep = frame.step; //in case cols != strides
    int nChannels = frame.channels();
    int sum;

    // Setup a rectangle to define your region of interest
    Rect myROI(160, 0, 320, 240); // start drawing a rect from top-left 
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

    unsigned char *cropData = cropped.data;
    int crop_width = cropped.cols;
    int crop_height = cropped.rows;
    int widthStepCrop = cropped.step; //in case cols != strides
    int nChannelsCrop = cropped.channels();

    int numOfPixelsCrop = crop_width*crop_height;
    int pixelValue[crop_width*crop_height], b=0, count = 0;
    double My, M;


    for(int i = 0; i < crop_height; i++){
      for(int j = 0; j < crop_width; j++){
        pixelValueRef[b] = (int)cropped_gray.at<uchar>(i,j);
        b++;
      }
    }

    int m = sizeof(pixelValue)/sizeof(pixelValue[0]);
    sort(pixelValue, pixelValue+m);

    /*cout << "Array after sorting : \n"; 
    for (int i = 0; i < n; ++i) 
      cout << arr[i] << " ";*/

    for (int p=0; p < 300; p++) {
      count = count + pixelValue[p];
    }

    My = count/300;

    M = Mx/My;




    for(int y=0;y<crop_height;y++)
    {
      for(int x=0;x<crop_width;x++)
      {
        // get pixel
        Vec3b color = cropped.at<Vec3b>(Point(x,y));

        color[0] = (int)(color[0]*M);
        color[1] = (int)(color[1]*M);
        color[2] = (int)(color[2]*M);

        // set pixel
        cropped.at<Vec3b>(Point(x,y)) = color;
    }
}





    for(int i = crop_height; i >= 0; i--){
      for(int j = 0; j < crop_width; j++){
        int blue = cropData[widthStepCrop*i + nChannelsCrop* j + 0];
        int green = cropData[widthStepCrop*i + nChannelsCrop* j + 1];
        int red = cropData[widthStepCrop*i + nChannelsCrop* j + 2];
        //cout << red << " " << green << " " << blue << endl;

        if (red >= thres_val) {
          if (blue >= thres_val) {
            if (green >= thres_val) {
              //printf("white line found at height: %d\n", i);
              sum = sum + 1;
              //printf("sum = %d\n", sum);
            }
          }
        }
      }
    }

    printf("number of pixels = %d,    sum = %d\n", numOfPixels, sum);
      
    if (sum > 10) {

      //left_speed_K = -SPEED;
      gpioPWM(en1, high); //p.ChangeDutyCycle(50)
      gpioWrite(in1, 0); //GPIO.output(in1,GPIO.LOW)
      gpioWrite(in2, 1); //GPIO.output(in2,GPIO.HIGH)
        
      //right_speed_K = -SPEED;
      gpioPWM(en2, high); //p.ChangeDutyCycle(50)
      gpioWrite(in3, 0); //GPIO.output(in3,GPIO.LOW)
      gpioWrite(in4, 1); //GPIO.output(in4,GPIO.HIGH);

        

    }
    else {
      //left_speed_K = SPEED;
      gpioPWM(en1, low); //p.ChangeDutyCycle(50)
      gpioWrite(in1, 1); //GPIO.output(in1,GPIO.HIGH)
      gpioWrite(in2, 0); //GPIO.output(in2,GPIO.LOW);

      //right_speed_K = SPEED;
      gpioPWM(en2, low); //p.ChangeDutyCycle(50)
      gpioWrite(in3, 1); //GPIO.output(in3,GPIO.HIGH)
      gpioWrite(in4, 0); //GPIO.output(in4,GPIO.LOW);
    }
      

  }


  gpioTerminate();
  return 0;

}
