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

enum BLOB_TYPE { RED, GREEN, BLUE, NONE };

int in1 = 24, in2 = 23, en1 = 25, temp1 = 1;
int in3 = 27, in4 = 22, en2 = 17, temp2 = 1;
int high = 192, medium = 128, low = 64; //max = 255

int main() {

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

  Mat frame;
  //--- INITIALIZE VIDEOCAPTURE
  VideoCapture cap(0);

  int pause_counter_K = 0;
  int left_speed_K, right_speed_K;
  int k, l;
  int red_K, blue_K, green_K;
  int spin_counter = 0, spinning = 0;
  
  /* Main loop */
  while (1){
        
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
    int widthStep = frame.step; //in case cols != strides
    int nChannels = frame.channels();

    /* Decrement the pause_counter*/
    if (pause_counter_K > 0)
      pause_counter_K--;

    else if (pause_counter_K > 10) {
      collision = 1;
     
    }

    else {  // pause_counter == 0

      /* Reset the sums */
      red_K = 0;
      green_K = 0;
      blue_K = 0;
      
      /*
       * Here we analyse the image from the camera. The goal is to detect a
       * blob (a spot of color) of a defined color in the middle of our
       * screen.
       * In order to achieve that we simply parse the image pixels of the
       * center of the image, and sum the color components individually
      */
      for(int i = cam_height / 2; i < cam_height; i++){
        for(int j = cam_width / 4; j < 3 * cam_width / 4; j++){
          int blue = myData[widthStep*i + nChannels* j + 0];
          int green = myData[widthStep*i + nChannels* j + 1];
          int red = myData[widthStep*i + nChannels* j + 2];
          //cout << red << " " << green << " " << blue << endl;

          blue_K = blue_K + blue;
          green_K = green_K + green;
          red_K = red_K + red;
        }
      }
      
      //printf("red_K = %d\n", red_K);
      //printf("green_K = %d\n", green_K);
      //printf("blue_K = %d\n", blue_K);

      if ((blue_K > 1.1 * red_K) && (blue_K > 1.1 * green_K)) {
        current_blob_K = BLUE;
      }

      else {
        current_blob_K = NONE;
      }

      if ((current_blob_K == NONE) && (collision == 0)) {
        //left_speed_K = -SPEED;
        gpioPWM(en1, medium); //p.ChangeDutyCycle(50)
        gpioWrite(in1, 0); //GPIO.output(in1,GPIO.LOW)
        gpioWrite(in2, 1); //GPIO.output(in2,GPIO.HIGH)
        
        //right_speed_K = SPEED;
        gpioPWM(en2, medium); //p.ChangeDutyCycle(50)
        gpioWrite(in3, 1); //GPIO.output(in3,GPIO.HIGH)
        gpioWrite(in4, 0); //GPIO.output(in4,GPIO.LOW);

      }

      else if ((current_blob_K != NONE) && (spinning == 0)) {
        //left_speed_K = SPEED;
        gpioPWM(en1, medium); //p.ChangeDutyCycle(50)
        gpioWrite(in1, 1); //GPIO.output(in1,GPIO.HIGH)
        gpioWrite(in2, 0); //GPIO.output(in2,GPIO.LOW);

        //right_speed_K = SPEED;
        gpioPWM(en2, medium); //p.ChangeDutyCycle(50)
        gpioWrite(in3, 1); //GPIO.output(in3,GPIO.HIGH)
        gpioWrite(in4, 0); //GPIO.output(in4,GPIO.LOW);

        spinning = 1;       
        pause_counter_K = 20;
        //printf("ok\n");
      }
      else if (spinning == 1) {
        //    left_speed_K = 2*SPEED;
        //left_speed_K = SPEED;
        gpioPWM(en1, high); //p.ChangeDutyCycle(75)
        gpioWrite(in1, 1); //GPIO.output(in1,GPIO.HIGH)
        gpioWrite(in2, 0); //GPIO.output(in2,GPIO.LOW);

        //right_speed_K = 0.5*SPEED;
        gpioPWM(en2, low); //p.ChangeDutyCycle(25)
        gpioWrite(in3, 1); //GPIO.output(in3,GPIO.HIGH)
        gpioWrite(in4, 0); //GPIO.output(in4,GPIO.LOW);

        spinning = 2;
        pause_counter_K = 15;
        //printf("hi\n");
        //printf("spin_counter_1 = %d\n", spin_counter);
      }
      else if (spinning == 2){
        //    left_speed_K = 0.5*SPEED;
        //left_speed_K = SPEED;
        gpioPWM(en1, low); //p.ChangeDutyCycle(25)
        gpioWrite(in1, 1); //GPIO.output(in1,GPIO.HIGH)
        gpioWrite(in2, 0); //GPIO.output(in2,GPIO.LOW)

        //right_speed_K = 2*SPEED;
        gpioPWM(en2, high); //p.ChangeDutyCycle(50)
        gpioWrite(in3, 1); //GPIO.output(in3,GPIO.HIGH)
        gpioWrite(in4, 0); //GPIO.output(in4,GPIO.LOW);

        spin_counter --;
        spinning = 0;
        pause_counter_K = 20;
        //printf("hey\n");
        //printf("spin_counter_2 = %d\n", spin_counter);
      }
    }
  }
  gpioTerminate();
  return 0;
}
