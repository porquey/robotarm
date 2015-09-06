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

void ReprojectPoints(Point3f pt, Point2f &pt1, Point2f &pt2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation);
void DetermineBasePairs(KeyPoint *random, KeyPoint *sorted);
void DeterminePairs(KeyPoint *random, KeyPoint *sorted, Point pt1, Point pt2);
//void CalculateLinkVector(vector<Point2f> points, vector<Point3f> linkPoints, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation);
void CalculateJoint(Point3f *link1, Point3f *link2, Point3f &joint, double &angle);
Point3f Calculate3DPoint(Point2f pt1, Point2f pt2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation);
double CalculateDisplacement(Point a, Point b);
double CalculateAngle(Point3f a, Point3f b);
Point3f CalculateVector(Point3f a, Point3f b);
double CalculateDotProduct(Point3f a, Point3f b);
Point3f CalculateCrossProduct(Point3f a, Point3f b);
int FindAngleDirection(Point3f baseVector, Point3f link1, Point3f link2);
double CalculateLength(Point3f a);
Point3f CalculateUnitVector(Point3f a, Point3f b);
Point3f MultiplyVector(Point3f a, double length);
Point2f Convert3fTo2f(Point3f a);

#endif /* defined(__robotarm__JointPositions__) */

