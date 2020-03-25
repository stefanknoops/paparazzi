/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include <opencv/cv.h>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/video/tracking_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "modules/computer_vision/farneback_cpp.h"


#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef TTC_FPS
#define TTC_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

//settings (hier de constantes definen



//hier de main functie


//hier de init (voor de mutexen)
void ttc_calc_init(void)  
{
	memset(ttc_final, 0, sizeof(float));
  	pthread_mutex_init(&mutex, NULL)
  	cv_add_to_device(&FARNEBACK_CAMERA, object_detector1, TTC_FPS); //tweede argument is?



void ttc_calc_periodic(void)
{
  static float ttc_final;
  pthread_mutex_lock(&mutex);
  memcpy(ttc_final, ttc, sizeof(float));
  pthread_mutex_unlock(&mutex);

  if(ttc_updated){
    AbiSendMsgVISUAL_DETECTION(FARNEBACK_AVOIDER_COLLISION_DETECTION, ttc_final);
    ttc_updated = false;
  }

  }
}

