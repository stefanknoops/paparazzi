
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


// Function
struct image_t *opencv_func(struct image_t *img);
struct image_t *opencv_func(struct image_t *img)
{
  //image_yuv422_downsample(img,img,16);

  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    struct flow_t *vector_ptr = opencv_example((char *) img->buf, img->w, img->h);

    int count = img->w * img->h;
    float error_threshold = 10.0;
    int n_iterations = 100 ;
    int n_samples = 1000;
    int im_width = img->w; //not sure about this, check how reference frame is defined
    int im_height = img->h;
    struct linear_flow_fit_info info;
    struct linear_flow_fit_info *info_ptr;
	info_ptr = &info;
    bool test = analyze_linear_flow_field(vector_ptr, count, error_threshold, n_iterations, n_samples, im_width, im_height, &info);
  }



// opencv_example(NULL, 10,10);

  return NULL;
}

void opencvdemo_init(void)
{
  cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func, 4);
}

