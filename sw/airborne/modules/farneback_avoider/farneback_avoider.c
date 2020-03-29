/*
 * Copyright (C) Max_Stefan
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/farneback_avoider/farneback_avoider.c"
 * @author Max_Stefan
 * Adaption of orange avoider in order to use the Farneback method
 */
#include "modules/farneback_avoider/farneback_avoider.h"
#include "modules/computer_vision/cv.h"

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "TTC_calculator.h"
#include "modules/computer_vision/lib/vision/image.h"
//#include "modules/farneback_avoider/Farneback_calculator.h"

#ifndef FARNEBACK_FPS
#define FARNEBACK_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(FARNEBACK_FPS)

#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};
// define and initialize global variables
enum navigation_state_t navigation_state = SAFE;
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]
//float ttc = 0;
float ttc_temp = 0;
float safe_time = 0;
float safe_time_threshold = 4.9;
int test_free_confidence = 5;
int TURNING = 0;
int turn_counter = 0;
int turnsize = 0;
float desired_heading = 0;
float error_in_heading;
float noise_level = 5.0f;

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

#define FARNEBACK_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[farneback_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if FARNEBACK_AVOIDER_VERBOSE

#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef FARNEBACK_AVOIDER_RECEIVER
#define FARNEBACK_AVOIDER_RECEIVER ABI_BROADCAST
#endif

static abi_event farneback_detection_ev;
static void farneback_detection_cb(int __attribute__((unused)) senderid, float ttc) //HIER DE TTC AANPASSEN (opencvexample)
		{
		  safe_time = ttc;
		}

void farneback_init(void) {
	  // Initialise random values
	  srand(time(NULL));
	  chooseRandomIncrementAvoidance();
	  printf("farneback init gelukt \n");
		 AbiBindMsgFARNEBACK_DETECTION(FARNEBACK_AVOIDER_RECEIVER, &farneback_detection_ev, farneback_detection_cb);
		 printf("ttc NA BIND MSG \n");


}

void farneback_periodic(struct image_t *img)
{
	// only evaluate our state machine if we are flying
	if(!autopilot_in_flight()){
    return;
  };



  VERBOSE_PRINT("Safe_time: %f  threshold: %f state: %d \n", safe_time, safe_time_threshold, navigation_state);

  // update our safe confidence using color thresholdF
  if (TURNING == 1){
	  safe_time = noise_level;
  }

  if(safe_time > safe_time_threshold){
    test_free_confidence++;
  } else {
    test_free_confidence -= 3;  // be more cautious with positive obstacle detections
  }
  Bound(test_free_confidence, 0, max_trajectory_confidence);

  if (test_free_confidence ==0){
	  printf("POLE DETECTED\n");
  }

  obstacle_free_confidence = test_free_confidence;
  // bound obstacle_free_confidence

  float moveDistance = fminf(maxDistance, 0.10f * obstacle_free_confidence);


  switch (navigation_state){
    case SAFE:

    	// check if drone is turning
    	if (TURNING ==1 ){
    		 // Wait after turning to reduce oscillations drone
    		 if (turn_counter >= 45){ //wait for the turning motion to be 100% done
    			 TURNING = 0;
    		     turn_counter = 0;
    		 }
    		 else {
    		     turn_counter++;
    		 }
    	}
    	// If not turning, move waypoint forward
    	else if(TURNING ==0){
    	moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
			if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
				navigation_state = OUT_OF_BOUNDS;
			}
			else if (obstacle_free_confidence == 0){
				navigation_state = OBSTACLE_FOUND;
			}
			else {
				moveWaypointForward(WP_GOAL, moveDistance);
			}
    	}

      break;
    case OBSTACLE_FOUND:
    	printf("OBSTACLE FOUND TRUE");

      // stop
      waypoint_set_here_2d(WP_GOAL);
      waypoint_set_here_2d(WP_TRAJECTORY);
      moveWaypointForward(WP_TRAJECTORY, -2.5f);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      // turnsize 18 means: 18*5 = 90 degrees turn
      turnsize = 19;

      break;
    case SEARCH_FOR_SAFE_HEADING:

    	// already turning?
		if (TURNING == 0) {
			increase_nav_heading(turnsize*heading_increment);
			TURNING = 1;
			printf("Started turning");
		}
		// check if desired turn angle achieved
		else if (error_in_heading < 2.0f){
			navigation_state = SAFE;
			test_free_confidence = 5;
			printf("Stopped turning");

		}

		// error between desired heading and current heading
		float temp_error_in_heading = DegOfRad(desired_heading - stateGetNedToBodyEulers_f()->psi);
		error_in_heading = abs(fmod(temp_error_in_heading + 180.0f, 360.0f) -180.0f);

		break;
    case OUT_OF_BOUNDS:
     //stop
      waypoint_set_here_2d(WP_GOAL);
      waypoint_set_here_2d(WP_TRAJECTORY);

      moveWaypointForward(WP_TRAJECTORY, 0.5f);

      // reset safe counter
      obstacle_free_confidence = 0;

      // ensure direction is safe before continuing
      navigation_state = SEARCH_FOR_SAFE_HEADING;

      // define turnsize: 26 * 5 = 130 degrees
      turnsize = 26;
      //}
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
   desired_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(desired_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(desired_heading);

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(desired_heading));
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{

  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
               // POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
               // stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                //POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{

  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = 5.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  printf("choose random increment gelukt \n");

  return false;

}




