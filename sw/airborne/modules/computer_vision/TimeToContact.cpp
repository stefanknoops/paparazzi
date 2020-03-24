#include "TimeToContact.h"


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


struct flow_t* opencv_example(char *img, int width, int height)
{
	 // Create a new image, using the original bebop image.
	 Mat M(height, width, CV_8UC2, img);
	 Mat image;
	 //Mat image1 = imread("/home/daan/github/sw/airborne/modules/computer_vision/frame1.jpg", 0);
	 //Mat image2 = imread("/home/daan/github/sw/airborne/modules/computer_vision/frame2.jpg", 0);


	 //  Grayscale image example
	 cvtColor(M, image, CV_YUV2GRAY_Y422);
	 //resize(image,image,Size(60,130),0,0,INTER_NEAREST);


	 if (prev_image.empty()){
		 prev_image = image;
	 	 }

	 Mat flow, cflow, frame;
	 Mat gray, prevgray, uflow;

	 calcOpticalFlowFarneback(prev_image, image, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);
	 //calcOpticalFlowFarneback(image1, image2, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);
	 prev_image = image;

	 float c1 = uflow.at<Vec2f>(50,50)[0];
	 printf(" %f \n", c1);
	 // fill struct with flow vectors
	 struct flow_t vectors[uflow.cols * uflow.rows];
	 struct flow_t *vectors_ptr;
	 vectors_ptr = vectors;

	 //optimise this if necessary
	 for (int i=0 ; i< (uflow.rows); i++){
		 for(int j = 0; j < (uflow.cols) ;j++){
		 vectors_ptr[j + i * uflow.cols].pos.x = j;
		 vectors_ptr[j + i * uflow.cols].pos.y = i;
		 vectors_ptr[j + i * uflow.cols].flow_x = uflow.at<Vec2f>(i,j)[0];
		 vectors_ptr[j + i * uflow.cols].flow_y = uflow.at<Vec2f>(i,j)[1];
		 }
	 }
	 /*
	 //printf("flow in x: %d \n", vectors_ptr[23860].flow_x);
	 //struct flow_t *vectors_ptr = &vectors;
	 //struct flow_t *vectors_ptr;
	 int count = int(uflow.cols * uflow.rows);
	 float error_threshold = 10.0;
	 int n_iterations = 100 ;
	 int n_samples = 25;
	 int im_width = uflow.cols; //not sure about this, check how reference frame is defined
	 int im_height = uflow.rows;
	 struct linear_flow_fit_info *info;
	 analyze_linear_flow_field(vectors_ptr, count, error_threshold, n_iterations, n_samples, im_width, im_height, info);
	*/
	return vectors_ptr;
}
