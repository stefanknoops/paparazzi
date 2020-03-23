#include "TimeToContact.h"


using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>
#include <iostream>
#include <modules/computer_vision/lib/vision/image.h>

using namespace cv;
Mat* prev_image;


int opencv_example(char *img, int width, int height)
{
	 // Create a new image, using the original bebop image.
	 Mat M(height, width, CV_8UC2, img);
	 Mat image;

	 //  Grayscale image example
	 cvtColor(M, image, CV_YUV2GRAY_Y422);
std::cout << *img << endl;
std::cout << img;
	
	resize(image,image,Size(60,130),0,0,INTER_NEAREST);
	 if (prev_image == NULL){
		 prev_image = &image;
	 	 }

	 printf("%d", 5);
	 Mat flow, cflow, frame;
	 UMat gray, prevgray, uflow;


	 calcOpticalFlowFarneback(*prev_image, image, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);

	 prev_image = &image;

	 //float test = uflow.x;
	std::cout << uflow.cols <<endl;
	std::cout << uflow.rows <<endl;
	return 0;
}
