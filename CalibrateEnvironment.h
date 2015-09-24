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
#define SQUARE_SIZE 40
#define BOARD_WIDTH 9
#define BOARD_HEIGHT 6
#define ITERATIONS 10

//Holds all information about the physical chessboard used
class BoardSettings{
public:
    int cornerNum;
    int squareSize;
    Size boardSize;
};

bool RetrieveChessboardCorners(vector<vector<Point2f> >& imagePoints1, vector<vector<Point2f> >& imagePoints2, BoardSettings s, VideoCapture inputCapture1, VideoCapture inputCapture2, int iterations);

void CalcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners);

void CalibrateEnvironment(VideoCapture& inputCapture1, VideoCapture& inputCapture2);
