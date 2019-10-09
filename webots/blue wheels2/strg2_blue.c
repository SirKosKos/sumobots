/*
 * Copyright 1996-2018 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  An example of use of a camera device.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/utils/system.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

#define SPEED 2
#define TIME_STEP 64
enum BLOB_TYPE { RED, GREEN, BLUE, NONE };

int main() {
  const char *robot_name[2] = {"KOSTAS", "ANTONIA"};
  WbDeviceTag camera_A, left_motor_A, right_motor_A;
  int width_A, height_A;
  int pause_counter_A = 0, turn_counter_A = 0;
  int left_speed_A, right_speed_A;
  int m, n; //k,l
  int red_A, blue_A, green_A;
  int blob_found_A = 0;
  int collision_A = 0;
  enum BLOB_TYPE current_blob_A;
  double dist_A = 0.0;
  int spin_counter = 0, spinning = 0;
  
  wb_robot_init();
  
  // get handle to robot's translation field
  WbNodeRef robot_node = wb_supervisor_node_get_from_def(robot_name[0]);
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  //const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
  //double dist = 0;
  
  /* Get the camera device, enable it, and store its width and height */
  camera_A = wb_robot_get_device("camera A");
  wb_camera_enable(camera_A, TIME_STEP);
  width_A = wb_camera_get_width(camera_A);
  height_A = wb_camera_get_height(camera_A);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor_A = wb_robot_get_device("left wheel motor A");
  right_motor_A = wb_robot_get_device("right wheel motor A");
  wb_motor_set_position(left_motor_A, INFINITY);
  wb_motor_set_position(right_motor_A, INFINITY);
  wb_motor_set_velocity(left_motor_A, 0.0);
  wb_motor_set_velocity(right_motor_A, 0.0);

  /* Main loop */
  while ((wb_robot_step(TIME_STEP) != -1) || (dist_A > 1)){
        
    /* Get the new camera values */
    const unsigned char *image_A = wb_camera_get_image(camera_A);
    
    // this is done repeatedly
    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
    //printf("%s is at position: %g %g %g\n", robot_name[1], trans[0], trans[1], trans[2]);
    
    // compute travelled distance
    double dist_A = sqrt(trans[0] * trans[0] + trans[2] * trans[2]);
    //printf("dist=%g\n", dist);
    
    /* Decrement the pause_counter*/
    if (pause_counter_A > 0)
      pause_counter_A--;

    /*
     * Case 1
     * A blob was found recently
     * The robot waits in front of it until pause_counter
     * is decremented enough
     */
    /*if (pause_counter_K > 10) {
      left_speed_K = SPEED;
      right_speed_K = SPEED;
    }*/
    /*
     * Case 2
     * A blob was found quite recently
     * The robot begins to turn but don't analyse the image for a while,
     * otherwise the same blob would be found again
     */
    else if (pause_counter_A > 0) {
      collision_A = 1;
     
    }
    /*
     * Case 3
     * The robot turns and analyse the camera image in order
     * to find a new blob
     */
    else {  // pause_counter == 0

      /* Reset the sums */
      red_A = 0;
      green_A = 0;
      blue_A = 0;
      //printf("pause_counter_K = %d\n", pause_counter_K);
      
      /*
       * Here we analyse the image from the camera. The goal is to detect a
       * blob (a spot of color) of a defined color in the middle of our
       * screen.
       * In order to achieve that we simply parse the image pixels of the
       * center of the image, and sum the color components individually
      */
      for (m = width_A / 4; m < 3 * width_A / 4; m++) {
        for (n = height_A / 2; n < height_A; n++) {
          red_A += wb_camera_image_get_red(image_A, width_A, m, n);
          blue_A += wb_camera_image_get_blue(image_A, width_A, m, n);
          green_A += wb_camera_image_get_green(image_A, width_A, m, n);
          
        }
      }
      
      //printf("red_K = %d\n", red_K);
      //printf("green_K = %d\n", green_K);
      //printf("blue_K = %d\n", blue_K);

      /*
       * If a component is much more represented than the other ones,
       * a blob is detected
       */     
      /*if ((red_K > 1.15 * green_K) && (red_K > 1.15 * blue_K)) {
        current_blob_K = RED;
      }
      else if ((green_K > 1.15 * red_K) && (green_K > 1.15 * blue_K)) {
        current_blob_K = GREEN;
      }*/
      if ((green_A > 1.1 * red_A) && (green_A > 1.1 * blue_A)) {
        current_blob_A = GREEN;
      }
      else {
        current_blob_A = NONE;
      }
       /*
       * Case 3a
       * No blob is detected
       * the robot continues to turn
       */
      if ((current_blob_A == NONE) && (collision_A == 0)) {
        left_speed_A = -SPEED;
        right_speed_A = SPEED;
      }
      /*else if ((current_blob_K == NONE) && (collision == 1)) {
        left_speed_K = -2*SPEED;
        right_speed_K = 2*SPEED;
      }*/
      /*
       * Case 3b
       * A blob is detected
       * the robot stops, stores the image, and changes its state
       */
      else if ((current_blob_A != NONE) && (spinning == 0)) {
        left_speed_A = SPEED;
        right_speed_A = SPEED;
        spinning = 1;       
        pause_counter_A = 20;
        //printf("ok\n");
      }
      else if (spinning == 1) {
        left_speed_A = 2*SPEED;
        right_speed_A = 0.5*SPEED;
        spinning = 2;
        pause_counter_A = 15;
        //printf("hi\n");
        //printf("spin_counter_1 = %d\n", spin_counter);
      }
      else if (spinning == 2){
        left_speed_A = 0.5*SPEED;
        right_speed_A = 2*SPEED;
        spin_counter --;
        spinning = 0;
        pause_counter_A = 20;
        //printf("hey\n");
        //printf("spin_counter_2 = %d\n", spin_counter);
      }
      /*else {
        left_speed_K = 2*SPEED;
        right_speed_K = 2*SPEED;
        pause_counter_K = 20;
      }*/
    }

    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor_A, left_speed_A);
    wb_motor_set_velocity(right_motor_A, right_speed_A);
    
    if (dist_A > 1.07) {
      wb_motor_set_velocity(left_motor_A, 0);
      wb_motor_set_velocity(right_motor_A, 0);
      
      break;
    }
  }
  /* Set the motor speeds for "Kostas". */
  //wb_motor_set_velocity(left_motor_K, 0);
  //wb_motor_set_velocity(right_motor_K, 0);
  printf("DONE");
  /* Set the motor speeds for "Antonia". */
  //wb_motor_set_velocity(left_motor_A, 0);
  //wb_motor_set_velocity(right_motor_A, 0);
  
  wb_robot_cleanup();

  return 0;
}
