//
//  JointPositions.h
//  robotarm
//
//  Created by Forest Fraser on 31/08/15.
//  Copyright (c) 2015 UoA. All rights reserved.
//

#ifndef __robotarm__JointPositions__
#define __robotarm__JointPositions__

#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

bool DeterminePairs(vector<KeyPoint> &random, vector<KeyPoint> &sorted, double mag);

void CalculateJoints(vector<Point2f>& pixels, Mat cameraMatrix1, Mat cameraMatrix2, Mat distance);
Point3f Calculate3DPoint(Point2f pixels1, Point2f pixels2, Mat cameraMatrix1, Mat cameraMatrix2, Mat distance);

#endif /* defined(__robotarm__JointPositions__) */

