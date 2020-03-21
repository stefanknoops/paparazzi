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

#ifndef FARNEBACK_FPS
#define FARNEBACK_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(FARNEBACK_FPS)

#include "modules/farneback_avoider/farneback_avoider.h"
#include "modules/computer_vision/cv.h"
#include "modules/orange_avoider/farneback.hpp" //staat nu wel op een gekke plek maar prima // we moeten deze wel nog maken
#include "modules/computer_vision/opticflow/linear_flow_fit.h"
#include "pthread.h"

struct image_t *prev_img;
float ttc;
float ttc_temp;

static pthread_mutex_t mutex;

float error_threshold = 0.25;
int n_iterations = 1;
int n_samples = 3;
int im_width = front_camera.output_size.w; //dit werkt wss niet
int im_height = front_camera.output_size.h;
struct linear_flow_fit_info *info;
  

static float call_ttc(struct image_t *img) { //video callback function
  struct flow_t *vectors = farneback_func(struct image_t *img,struct image_t *prev_img) 
  //linear_flow_fit
  if analyze_linear_flow_field(struct flow_t *vectors, int count, float error_threshold, int n_iterations, int n_samples, int im_width, int im_height, struct linear_flow_fit_info *info)
  {
  ttc_temp = info->time_to_contact;
  }
  else
  {
  //wat als hij geen fit vind?
  }
  
  pthread_mutex_lock(&mutex);
  ttc  =  ttc_temp;
  pthread_mutex_unlock(&mutex);
  
  *prev_img = *img;
  return ttc_temp //niet zeker maar voor nu
}

void farneback_init(struct image_t *img) {
  //pas airframe aan in xml
  cv_add_to_device(&FARNEBACK_CAMERA, call_ttc, FARNEBACK_FPS);
  *prev_img = *img;
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb); //dit gebruiken om hem in orange avoider te implementen

  memset(ttc, 0, sizeof(float));
  pthread_mutex_init(&mutex, NULL);
}



void farneback_periodic()
{
 

  //de flow met een threshold vergelijken


  //de navigatie kopieren van orange_avoider

  

  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}


