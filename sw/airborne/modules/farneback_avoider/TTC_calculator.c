// Own header

#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include <opencv/cv.h>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/video/tracking_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "Farneback_calculator.h"
#include "modules/computer_vision/lib/vision/image.h"


#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"


//nog aanpassen
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

//settings (hier de constanten definen)
float error_threshold = 10.0;
int n_iterations = 100;
int n_samples = 1000;

struct image_t *img;
//define global variables
//geen nieuwe gedefinieerd (staan allemaal al in image.h of op andere plekken)

float *ttc = 0;
float *ttc_glob = 0;

//hier de main functie
float ttc_calculator_func(void)
{
  //image_yuv422_downsample(img,img,16);
  if (img->type == IMAGE_YUV422) {
   	// Call OpenCV (C++ from paparazzi C function)
    	struct flow_t *vector_ptr = farneback_flow((char *) img->buf, img->w, img->h); //deze functie moet vervangen worden door de uiteindelijk
    	int count = img->w * img->h;
    	int im_width = img->w; //not sure about this, check how reference frame is defined
    	int im_height = img->h;
    	struct linear_flow_fit_info info;
    	struct linear_flow_fit_info *info_ptr;
    	info_ptr = &info;
    	bool test = analyze_linear_flow_field(vector_ptr, count, error_threshold, n_iterations, n_samples, im_width, im_height, &info);
    	*ttc = info->time_to_contact;
  }
  //return ttc;
}

//hier de init (voor de mutexen)
void ttc_calc_init(void)  
{
	memset(*ttc_glob, 0, sizeof(float));
  	pthread_mutex_init(&mutex, NULL);
  	cv_add_to_device(&FARNEBACK_CAMERA, img, TTC_FPS); //tweede argument is volgens mij gewoon de afbeelding


}
void ttc_calc_periodic(void)
{

  //float ttc_final;
  pthread_mutex_lock(&mutex);
  memcpy(*ttc_glob, *ttc, sizeof(float));
  pthread_mutex_unlock(&mutex);

}

