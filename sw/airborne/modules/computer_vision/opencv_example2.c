
// void opencvdemo_init() {}

/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/opencv_example2.h"
#include "modules/computer_vision/TimeToContact.h"
#include <stdio.h>
#include "modules/computer_vision/opticflow/linear_flow_fit.h"



#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCVDEMO_FPS)

#ifndef size_smooth
#define size_smooth 6
#endif

float history_ttc[size_smooth];

float EWMA(float *history_ttc, int size_history, float degree_of_decrease)
{
	//printf("%f", history_ttc[0]);
	float num_ttc = 0.0f;
	float den_ttc = 0.0f;
	for (int i = 0; i<(size_history); i++){
		num_ttc += history_ttc[i]*pow((1-degree_of_decrease),i);
		//printf("num is: %f \n", num_ttc);
		den_ttc += pow((1-degree_of_decrease),i);
		//printf("den is: %f \n", den_ttc);
	}
	return (num_ttc/den_ttc);
}


// Function
struct image_t *opencv_func(struct image_t *img);
struct image_t *opencv_func(struct image_t *img)
{
  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    struct flow_t *vector_ptr = opencv_example((char *) img->buf, img->w, img->h);
	int count = 60*60 / 9.0f ;
	float error_threshold = 10.0;
	int n_iterations = 100 ;
	int n_samples = 50;
	int im_width = 60; //not sure about this, check how reference frame is defined
	int im_height = 60;
	struct linear_flow_fit_info info;
	struct linear_flow_fit_info *info_ptr;
	info_ptr = &info;

	bool test = analyze_linear_flow_field(vector_ptr, count, error_threshold, n_iterations, n_samples, im_width, im_height, &info);
    float ttc = info.time_to_contact;

    ttc = abs(ttc)*1.0f/20.0f;
    printf("%f \n", ttc);

    if (history_ttc[0] == 0.0f){
    	if(ttc > 5){
    		ttc = 5.0f;
    	}
    	for (int i=0 ; i< (size_smooth); i++){
    		history_ttc[i] = ttc;
    	}
    }
    else{
    	if(ttc > 5){
    		if (history_ttc[0] < 3){
    			ttc = history_ttc[0];
    		}
    		else{
    			ttc = history_ttc[0];
    		}
    	}
    	for (int i=size_smooth; i>1;i -=1){
    		history_ttc[i-1] = history_ttc[i-2];
    	}
    	history_ttc[0] = ttc;
    }
    float smooth_ttc = EWMA(&history_ttc,size_smooth,0.6);


  }
    // opencv_example(NULL, 10,10);
    //memset(history_ttc, 1, 5*sizeof(history_ttc[0]));




  return NULL;
}

void opencvdemo_init(void)
{
  cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func, 20);
}

