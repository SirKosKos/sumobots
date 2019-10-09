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
  WbDeviceTag camera_K, left_motor_K, right_motor_K;
  const char *robot_name[2] = {"KOSTAS", "ANTONIA"};
  int width_K, height_K;
  int pause_counter_K = 0;
  int left_speed_K, right_speed_K;
  int i, j;
  int red_K, blue_K, green_K;
  int found = 0;
  enum BLOB_TYPE current_blob_K;
  int static_counter = 0;

  wb_robot_init();
  
  // get handle to robot's translation field
  WbNodeRef robot_node_K = wb_supervisor_node_get_from_def(robot_name[1]);
  WbFieldRef trans_field_K = wb_supervisor_node_get_field(robot_node_K, "translation");

  /* Get the camera device, enable it, and store its width and height */
  camera_K = wb_robot_get_device("camera K");
  wb_camera_enable(camera_K, TIME_STEP);
  width_K = wb_camera_get_width(camera_K);
  height_K = wb_camera_get_height(camera_K);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor_K = wb_robot_get_device("left wheel motor K");
  right_motor_K = wb_robot_get_device("right wheel motor K");
  wb_motor_set_position(left_motor_K, INFINITY);
  wb_motor_set_position(right_motor_K, INFINITY);
  wb_motor_set_velocity(left_motor_K, 0.0);
  wb_motor_set_velocity(right_motor_K, 0.0);

  /* Main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    /* Get the new camera values */
    const unsigned char *image_K = wb_camera_get_image(camera_K);
    
    // this is done repeatedly
    const double *trans_K = wb_supervisor_field_get_sf_vec3f(trans_field_K);
    //printf("%s is at position: %g %g %g\n", robot_name[1], trans[0], trans[1], trans[2]);
    
    // compute travelled distance
    double dist_K = sqrt(trans_K[0] * trans_K[0] + trans_K[2] * trans_K[2]);
    //printf("dist_K = %g\n", dist_K);
    /* Decrement the pause_counter*/
    if (pause_counter_K > 0){
      pause_counter_K--;
      }
    
    if(pause_counter_K == 49){
      static_counter++;
    }
    /*if (pause_counter_A < 50 && pause_counter_A > 48){
      left_speed_A = SPEED;
      right_speed_A = -SPEED;
      printf("pause_counter_A = %d\n", pause_counter_A);
    }*/


    /*
     * Case 1
     * A blob was found recently
     * The robot waits in front of it until pause_counter
     * is decremented enough
     */
    if (static_counter < 15 && found == 0){
      if ( pause_counter_K <= 50 && pause_counter_K > 40) {
        left_speed_K = SPEED;
        right_speed_K = SPEED;
        //printf("pause_counter_A = %d\n", pause_counter_A);
      }
    
      if (pause_counter_K == 40){
        found = 1;
      }
    }

    if (static_counter >= 15 && found == 0){
      left_speed_K = SPEED;
      right_speed_K = -SPEED;
      static_counter = 0;
    }
    /*
     * Case 2
     * A blob was found quite recently
     * The robot begins to turn but don't analyse the image for a while,
     * otherwise the same blob would be found again
     */
    /*else if (pause_counter_A > 0) {
      left_speed_A = -SPEED;
      right_speed_A = SPEED;
    }*/
    /*
     * Case 3
     * The robot turns and analyse the camera image in order
     * to find a new blob
     */
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
      for (i = width_K / 3; i < 2 * width_K / 3; i++) {
        for (j = height_K / 2; j < height_K; j++) {
          red_K += wb_camera_image_get_red(image_K, width_K, i, j);
          blue_K += wb_camera_image_get_blue(image_K, width_K, i, j);
          green_K += wb_camera_image_get_green(image_K, width_K, i, j);
        }
      }

      /*printf("red_A = %d\n", red_A);
      printf("green_A = %d\n", green_A);
      printf("blue_A = %d\n", blue_A); */

      /*
       * If a component is much more represented than the other ones,
       * a blob is detected
       */
      if ((blue_K > 1.2 * red_K) && (blue_K > 1.2 * green_K))
        current_blob_K = BLUE;
      else
        current_blob_K = NONE;

      /*
       * Case 3a
       * No blob is detected
       * the robot continues to turn
       */
      if (current_blob_K == NONE && found == 0 && pause_counter_K < 45) {
        left_speed_K = SPEED;
        right_speed_K = -SPEED;
      }

      if (current_blob_K == NONE && found == 1 && pause_counter_K < 45) {
        left_speed_K = -SPEED;
        right_speed_K = SPEED;
      }
      /*
       * Case 3b
       * A blob is detected
       * the robot stops, stores the image, and changes its state
       */
      if (current_blob_K == BLUE) {
        /*left_speed_A = SPEED;
        right_speed_A = SPEED;*/
        pause_counter_K = 50;
        found = 0;
      }
    }

    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor_K, left_speed_K);
    wb_motor_set_velocity(right_motor_K, right_speed_K);
    
    if (dist_K > 1.07) {
      wb_motor_set_velocity(left_motor_K, 0);
      wb_motor_set_velocity(right_motor_K, 0);
      printf("Kostas wins");
      break;
    }
  }

  wb_robot_cleanup();

  return 0;
}
