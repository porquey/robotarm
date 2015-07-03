#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

#define CAPTURE_DELAY 1000
#define SQUARE_SIZE 36.6
#define SQUARE_SIZE_SMALL 25.2
#define BOARD_WIDTH 9
#define BOARD_HEIGHT 6

void CalibrateEnvironment(VideoCapture& inputCapture1, VideoCapture& inputCapture2);
