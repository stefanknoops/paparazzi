#include <stdio.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"

using namespace std;
#include <opencv2/core/core.hpp>			//Hier staan de cart2polar en normalize in
#include <opencv2/cudaoptflow/include/optflow.hpp>
#include "opencv2/video/tracking.hpp"		//Hier staat de farneback methode in
#include "opencv2/imgproc/imgproc.hpp"		//Hier staan de resize en cvtcolor in
// hier moet de juiste opencv functie nog worden toegevoegd


#ifndef FARNEBACK_FPS
#define FARNEBACK_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(FARNEBACK_FPS)

#ifndef FARNEBACK_SEND_OBSTACLE
#define FARNEBACK_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(FARNEBACK_SEND_OBSTACLE)

struct video_listener *listener = NULL;

// hier waardes initialiseren
int fb_thresh = 10;
int optical_flow = 0;

// het vorige beeld en het huidige beeld moet worden ingeladen ergens (volgens mij dit)
struct prev = ;//vorige afbeelding
struct current = ;//huidige afbeelding
double pyr_scale = 0.5; //image scale for pyramids <1
int levels = 1; //number of pyramid layers (?) where 1 means that there are no extra layers
int winsize = ;// window size
int iterations = 1;
int poly_n = 5; //size of the pixel neighborhood used to find polynomial expansion in each pixel
double poly_sigma = 1.1 //standard deviation of the Gaussian that is used to smooth derivatives used as a basis for the polynomial expansion;

// hier farneback doen
// waardes van Daan voor nu gebruiken?
void calcOpticalFlowFarneback(InputArray prev, InputArray next, InputOutputArray flow, double pyr_scale, int levels, int winsize, int iterations, int poly_n, double poly_sigma) {
    flow_im = flow;
}


// hier treshold checken



// hier actie ondernemen?
