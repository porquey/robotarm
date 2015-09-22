#ifndef SET_BLOB_COLOUR_H
#define SET_BLOB_COLOUR_H



#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "HSVRange.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace std;
using namespace cv;


static int iLowH = 0;
static int iHighH = 179;

static int iLowS = 0;
static int iHighS = 255;

static int iLowV = 0;
static int iHighV = 255;

static int erosion = 3;
static int dilation = 3;

void MouseCallBack(int event, int x, int y, int flags, void* userData);
void SetBlobColour(VideoCapture& inputCapture1, VideoCapture& inputCapture2);
void ApplyMorphologicalOperation(Mat &img, int erosionSize, int dilationSize);

#endif