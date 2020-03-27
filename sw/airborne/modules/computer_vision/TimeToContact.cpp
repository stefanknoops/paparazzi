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
String path("/home/daan/Documents/AE4317_2019_datasets/Pictures/*.jpg"); //select only jpg
vector<String> fn;
int ind = 0;


struct flow_t* opencv_example(char *img, int width, int height)
{
	// import large test data set
	 vector<Mat> data;
	 glob(path,fn, true); // recurse
	 sort(fn.begin(), fn.end());
	 for (size_t k=0; k<fn.size(); ++k)
	 {
	     Mat im = imread(fn[k],CV_LOAD_IMAGE_GRAYSCALE);
	     //cout << outt << endl;
	     //Mat cropped_image = im;
	     Mat cropped_image = im(Rect(0,125,240,240));
	     if (im.empty()) continue; //only proceed if sucsessful
	     data.push_back(cropped_image);
	 	 }


	 for (size_t k =0; k < data.size(); k ++){
		 resize(data[k],data[k],Size(),0.25,0.25,INTER_AREA);
	 }
	 // Create a new image, using the original bebop image, use this when camera is taken as input
	 // Mat M(height, width, CV_8UC2, img);
	 // Mat image;
	 // cvtColor(M, image, CV_YUV2GRAY_Y422);
	 // if (prev_image.empty()){
	//	 prev_image = image;
	 //	 }

	 Mat flow, cflow, frame;
	 Mat gray, prevgray, uflow;

	 // import small test data set
	 Mat image1_temp = data[ind];//imread("/home/daan/Documents/AE4317_2019_datasets/Pictures/31537000.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	 Mat image2_temp = data[ind +1];//imread("/home/daan/Documents/AE4317_2019_datasets/Pictures/31639000.jpg", CV_LOAD_IMAGE_GRAYSCALE);

     //resize(image1_temp, image1_temp, Size(),0.25,0.25,INTER_AREA);
     //resize(image2_temp, image2_temp, Size(),0.25,0.25,INTER_AREA);

	 //Mat image1_temp_cropped = image1_temp(Rect(0,125,240,240));
	 //Mat image2_temp_cropped = image2_temp(Rect(0,125,240,240));

	 // apply farneback to find dense optical flow
	 //calcOpticalFlowFarneback(prev_image, image, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);
	 calcOpticalFlowFarneback(image1_temp, image2_temp, uflow, 0.5, 3, 15, 10, 7, 1.2, 0);
	 //prev_image = image;

	 // fill struct with flow vectors
	 struct flow_t vectors[uflow.cols * uflow.rows];
	 struct flow_t *vectors_ptr;
	 vectors_ptr = vectors;

	 //printf("rows modulo 3 \t %d", uflow.rows%3);

	 int k = 0;
	 int z = 0;
	 int vector_count = 0;
	 int subpixel_factor = 1;
	 for (int i=0 ; i< (uflow.rows); i = i + 3){
		 for(int j = 0; j < (uflow.cols) ;j = j +3){
		 vectors_ptr[k + z * uflow.cols].pos.x = (uint32_t)(j * subpixel_factor);
		 vectors_ptr[k + z * uflow.cols].pos.y = (uint32_t)(i * subpixel_factor);
		 vectors_ptr[k + z * uflow.cols].flow_x = (float)(uflow.at<Vec2f>(i,j)[0]* subpixel_factor * -1);
		 vectors_ptr[k + z * uflow.cols].flow_y = (float)(uflow.at<Vec2f>(i,j)[1] * subpixel_factor * -1);
	 	 k = k + 1;
	 	 vector_count++;
		 }
		 z = z + 1;
		k = 0;
	 }

	 //printf("total number of vectors: %d", vector_count);
		// for(int j = 0; j < (uflow.cols) ;j++){
			// int i =225;
		 //printf("\t %f \n", vectors_ptr[j + i * uflow.cols].flow_x);
		 //}


	 ind++;
	return vectors_ptr;
}
