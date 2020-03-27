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
#include <opencv2/core/utility.hpp>
#include <algorithm>

using namespace cv;
using namespace std;
Mat prev_image;

struct flow_t* farneback_flow(char *img, int width, int height)
{	

	 // Create a new image, using the original bebop image, use this when camera is taken as input
	  Mat M(height, width, CV_8UC2, img);
	  Mat image;
	  cvtColor(M, image, CV_YUV2GRAY_Y422);
	  if (prev_image.empty()){
		 prev_image = image;
	 	 }

	 Mat flow, cflow, frame;
	 Mat gray, prevgray, uflow;


	 // apply farneback to find dense optical flow
	 calcOpticalFlowFarneback(prev_image, image, uflow, 0.5, 3, 15, 10, 7, 1.2, 0);
	 prev_image = image;

	 // fill struct with flow vectors
	 struct flow_t vectors[uflow.cols * uflow.rows];
	 struct flow_t *vectors_ptr;
	 vectors_ptr = vectors;


	 for (int i=0 ; i< (uflow.rows); i++){
		 for(int j = 0; j < (uflow.cols) ;j++){
		 vectors_ptr[j + i * uflow.cols].pos.x = (uint32_t)j;
		 vectors_ptr[j + i * uflow.cols].pos.y = (uint32_t)i;
		 vectors_ptr[j + i * uflow.cols].flow_x = (float)uflow.at<Vec2f>(i,j)[0];
		 vectors_ptr[j + i * uflow.cols].flow_y = (float)uflow.at<Vec2f>(i,j)[1];
		 }
	 }
	return vectors_ptr;
}
