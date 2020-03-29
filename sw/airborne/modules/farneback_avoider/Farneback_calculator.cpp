/*
 * Copyright (C) Daan,Max, Mitchell, Ruben, Stefan
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
 * @file "Farneback_calculator.cpp"
 * @author Daan, Max, Mitchell, Ruben en Stefan
 * Computation of optical flow vectors with Farneback from Bebop images
 */

#include "modules/farneback_avoider/Farneback_calculator.h"


using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>
#include <iostream>
#include "modules/computer_vision/opticflow/linear_flow_fit.h"

using namespace cv;
Mat prev_image;


struct flow_t* farneback_flow(char *img, int width, int height)
{	

	 // Create a new image, using the original bebop image, use this when camera is taken as input
	  Mat M(height, width, CV_8UC2, img);
	  Mat image;

	  // Convert image to grayscale
	  cvtColor(M, image, CV_YUV2GRAY_Y422);

	  // crop
	  Mat crop_image = image(Rect(0,110,240, 300));

	  Mat flow, cflow, frame;
	  Mat gray, prevgray, uflow;

	  //scale image
	  resize(crop_image, crop_image, Size(),0.25,0.25,INTER_AREA);

	  if (prev_image.empty()){
	  		 prev_image = crop_image;
	  	 }

	 // apply farneback to find dense optical flow
	 calcOpticalFlowFarneback(prev_image, crop_image, uflow, 0.5, 3, 15, 10, 7, 1.2, 0);

	 prev_image = crop_image;

	 // fill struct with flow vectors
	 struct flow_t vectors[uflow.cols * uflow.rows];
	 struct flow_t *vectors_ptr;
	 vectors_ptr = vectors;

	 //grid with flow vectors is filled, every row 1 in 3 points is taken, every column, one in 3 points is taken
	 int k = 0;
	 int z = 0;
	 int vector_count = 0;
	 int subpixel_factor = 1;
	 for (int i=0 ; i< (uflow.rows); i +=3){
		 for(int j = 0; j < (uflow.cols) ;j += 3){
		 vectors_ptr[k + z * uflow.cols].pos.x = (uint32_t)(j * subpixel_factor);
		 vectors_ptr[k + z * uflow.cols].pos.y = (uint32_t)(i * subpixel_factor);
		 vectors_ptr[k + z * uflow.cols].flow_x = (float)(uflow.at<Vec2f>(i,j)[0]* subpixel_factor * -1);
		 vectors_ptr[k + z * uflow.cols].flow_y = (float)(uflow.at<Vec2f>(i,j)[1]* subpixel_factor * -1);
		 k++;
		 vector_count++;
		 }
		 z++;
		 k=0;
	 }
	return vectors_ptr;
}
