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
	printf("Hier print hij nog I\n");

	 // Create a new image, using the original bebop image, use this when camera is taken as input
	  Mat M(height, width, CV_8UC2, img);
	  Mat image;
	  //resize(M,M,Size(60,130),0,0,INTER_NEAREST);
	  cvtColor(M, image, CV_YUV2GRAY_Y422);

	  Mat crop_image = image(Rect(0,125,240,240));

	  Mat flow, cflow, frame;
	  Mat gray, prevgray, uflow;

	  resize(crop_image, crop_image, Size(),0.25,0.25,INTER_AREA);

	  if (prev_image.empty()){
	  		 prev_image = image;
	  	 	 }

	 // apply farneback to find dense optical flow
	 calcOpticalFlowFarneback(prev_image, crop_image, uflow, 0.5, 3, 15, 10, 7, 1.2, 0);
	 prev_image = crop_image;

	 // fill struct with flow vectors
	 struct flow_t vectors[uflow.cols * uflow.rows];
	 struct flow_t *vectors_ptr;
	 vectors_ptr = vectors;


	 int k = 0;
	 int z = 0;
	 int vector_count = 1;
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
