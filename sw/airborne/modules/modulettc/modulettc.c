/*
 * Copyright (C) Group 5
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
 * @file "modules/modulettc/modulettc.c"
 * @author Group 5
 * Calculates the time to contact from farneback optical flow vectors
 */

//Includes other functions
#include "modules/modulettc/modulettc.h"

#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/opticflow/linear_flow_fit.h"
#include "modules/farneback_avoider/Farneback_calculator.h"

//Includes standard functions
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

//All define statements
#ifndef FARNEBACK_AVOIDER_COLLISION_DETECTION
#define FARNEBACK_AVOIDER_COLLISION_DETECTION ABI_BROADCAST
#endif

#ifndef TTC_FPS
#define TTC_FPS 20 ///< Default FPS (zero means run at camera fps)
#endif

#ifndef size_smooth
#define size_smooth 6
#endif

//begin mutex
static pthread_mutex_t mutex;


//settings (defining the constants)
//Linear flow fit constants
float error_threshold = 10.0;
int n_iterations = 100;
int n_samples = 50;

//Image structure
struct image_t *img;

//Constants for the Exponential weighted moving average
float history_ttc[size_smooth];
float real_last_ttc;
float degree_of_decrease = 0.5;

//Variables for data transfer between threads and modules
float ttc_glob2;
bool ttc_updated = false;

//Exponential weighted moving average function for smoothing out the ttc data
float EWMA(float *history_ttc, int size_history, float degree_of_decrease)
{
	float num_ttc = 0.0f;
	float den_ttc = 0.0f;
	for (int i = 0; i<(size_history); i++){
		num_ttc += history_ttc[i]*pow((1-degree_of_decrease),i);
		den_ttc += pow((1-degree_of_decrease),i);
	}
	return(num_ttc/den_ttc);
}

//Function obtaining video data and getting the ttc
struct image_t *calc_ttc(struct image_t *img);
struct image_t *calc_ttc(struct image_t *img)
{
  if (img->type == IMAGE_YUV422) {
   	// Call OpenCV (C++ from paparazzi C function)
    	struct flow_t *vector_ptr = farneback_flow((char *) img->buf, img->w, img->h); //deze functie moet vervangen worden door de uiteindelijk
    	int count = 60 * 60 / 9.0f;
    	int im_width = 60;
    	int im_height = 60;
    	struct linear_flow_fit_info info;
    	bool test = analyze_linear_flow_field(vector_ptr, count, error_threshold, n_iterations, n_samples, im_width, im_height, &info);
    	float vid_ttc = abs(info.time_to_contact)*(1.0f/TTC_FPS);

    	real_last_ttc = vid_ttc;

    	//loop for filling the history of ttc, and find error data
    	if (history_ttc[0] == 0.0f){
    		if(vid_ttc > 5){
    			vid_ttc = 5.0f;
    		}
    		for (int i = 0; i < (size_smooth); i++){
    			history_ttc[i] = vid_ttc;
    		}
    	}
    	//Loop for shifting the old data throughout the array
    	else{
    		if(vid_ttc > 5.0f){
    			if (real_last_ttc < 3.0f){
    				vid_ttc = history_ttc[0];
    			}
    			else{
    				vid_ttc = 5.0f;
    			}
    		}
    		for (int i=size_smooth; i>1; i -=1){
    			history_ttc[i-1] = history_ttc[i-2];
    		}
    		history_ttc[0] = vid_ttc;
    	}

    	float smooth_ttc = EWMA(&history_ttc,size_smooth,degree_of_decrease);
    	pthread_mutex_lock(&mutex);
    		ttc_glob2 = smooth_ttc;
    	pthread_mutex_unlock(&mutex);
    	ttc_updated = true;
  }
  return img;
}

void ttc_init(void)
{
  	pthread_mutex_init(&mutex, NULL);

  	cv_add_to_device(&FARNEBACK_CAMERA2, calc_ttc, TTC_FPS); //Link front camera to images
}

void ttc_periodic(void){
	float ttc;
	pthread_mutex_lock(&mutex);
	memcpy(&ttc, &ttc_glob2, sizeof(float));
	pthread_mutex_unlock(&mutex);

	if(ttc_updated){
		AbiSendMsgFARNEBACK_DETECTION(FARNEBACK_AVOIDER_COLLISION_DETECTION, ttc);
		ttc_updated = false;
	}
}


