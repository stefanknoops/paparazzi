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

#include "modules/modulettc/modulettc.h"

#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/opticflow/linear_flow_fit.h"
#include "modules/farneback_avoider/Farneback_calculator.h"


#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#ifndef FARNEBACK_AVOIDER_COLLISION_DETECTION
#define FARNEBACK_AVOIDER_COLLISION_DETECTION ABI_BROADCAST
#endif

static pthread_mutex_t mutex;

#ifndef TTC_FPS
#define TTC_FPS 15 ///< Default FPS (zero means run at camera fps)
#endif

//settings (hier de constanten definen)
float error_threshold = 10.0;
int n_iterations = 100;
int n_samples = 50;

struct image_t *img;

float ttc_glob2;
bool ttc_updated = false;
struct image_t *calc_ttc(struct image_t *img);
struct image_t *calc_ttc(struct image_t *img)
{
	//printf("Hier print hij nog 22\n");
	//printf("%d",img->h);
  //image_yuv422_downsample(img,img,16);
  if (img->type == IMAGE_YUV422) {
   	// Call OpenCV (C++ from paparazzi C function)
		//printf("Hier print hij nog 3\n");

    	struct flow_t *vector_ptr = farneback_flow((char *) img->buf, img->w, img->h); //deze functie moet vervangen worden door de uiteindelijk
    	//printf("Hier print hij nog 4\n");

    	int count = 60 * 60 / 9.0f;
    	int im_width = 60; //not sure about this, check how reference frame is defined
    	int im_height = 60;
    	struct linear_flow_fit_info info;
    	struct linear_flow_fit_info *info_ptr;
    	info_ptr = &info;
    	//printf("this works still \n");
    	bool test = analyze_linear_flow_field(vector_ptr, count, error_threshold, n_iterations, n_samples, im_width, im_height, &info);
    	//printf("it works");
    	printf("ttc voor pthread = %f \n", info.time_to_contact*(1.0f/15.0f));

    	//printf("hier print hij nog XII\n");
    	pthread_mutex_lock(&mutex);
    	  ttc_glob2 = info.time_to_contact*(1.0f/15.0f);
    	pthread_mutex_unlock(&mutex);
    	ttc_updated = true;
  }
	//printf("Hier print hij weer 5\n");

  return img;
}

void ttc_init(void)
{
	 printf("ttc init \n");

	//memset(ttc_glob2, 0, sizeof(float));
	 printf("ttc init 2 \n");

  	pthread_mutex_init(&mutex, NULL);
	 printf("ttc init 3 \n");

  	cv_add_to_device(&FARNEBACK_CAMERA2, calc_ttc, TTC_FPS); //tweede argument is volgens mij gewoon de afbeelding
	 printf("ttc init 4 \n");


}

void ttc_periodic(void){
//printf("print ie dit nog");
	float ttc;
  pthread_mutex_lock(&mutex);
  memcpy(&ttc, &ttc_glob2, sizeof(float));
  pthread_mutex_unlock(&mutex);
printf("ttc na pthread= %f \n", ttc);

if(ttc_updated){
    AbiSendMsgFARNEBACK_DETECTION(FARNEBACK_AVOIDER_COLLISION_DETECTION, ttc);
    ttc_updated = false;
    printf("ttc NA ABI=%f\n",ttc);

  }

}


