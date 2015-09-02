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

bool DetermineBasePairs(vector<KeyPoint> &random, vector<KeyPoint> &sorted);
bool DeterminePairs(vector<KeyPoint> &random, vector<KeyPoint> &sorted, Point pt1, Point pt2);
void CalculateJoints(vector<Point2f>& pixels, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation);
void CalculateLinkVector(vector<Point2f> points, vector<Point3f> linkPoints, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation);
void CalculateJoint(vector<Point3f> link1, vector<Point3f> link2, Point3f &joint, double &angle);
Point3f Calculate3DPoint(Point2f pixels1, Point2f pixels2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation);
double CalculateDisplacement(Point a, Point b);
double CalculateAngle(Point3f a, Point3f b);
Point3f CalculateVector(Point3f a, Point3f b);
double CalculateDotProduct(Point3f a, Point3f b);
double CalculateLength(Point3f a);

#endif /* defined(__robotarm__JointPositions__) */

