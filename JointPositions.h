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

// reprojects 3D points back to a set of 2D points on the images
void ReprojectPoints(Point3f pt, Point2f &pt1, Point2f &pt2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation);
// calculates the 3D point for a given set of 2D points
Point3f Calculate3DPoint(Point2f pt1, Point2f pt2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation);
// calculates the distance between 2 points
double CalculateDisplacement(Point a, Point b);
// calculate the angle between 2 vectors
double CalculateAngle(Point3f a, Point3f b);
// calculates the vector between 2 points
Point3f CalculateVector(Point3f a, Point3f b);
// calculates the dot product
double CalculateDotProduct(Point3f a, Point3f b);
// calculates the cross product
Point3f CalculateCrossProduct(Point3f a, Point3f b);
// finds the direction of a joint angle
int FindAngleDirection(Point3f baseVector, Point3f link1, Point3f link2);
// calculates the length of a vector
double CalculateLength(Point3f a);
// calculates the unit vector
Point3f CalculateUnitVector(Point3f a, Point3f b);
// multiplies a vector
Point3f MultiplyVector(Point3f a, double length);
// converts a 3D point to 2D
Point2f Convert3fTo2f(Point3f a);

//void DetermineBasePairs(KeyPoint *random, KeyPoint *sorted);
//void DeterminePairs(KeyPoint *random, KeyPoint *sorted, Point pt1, Point pt2);
//void CalculateJoint(Point3f *link1, Point3f *link2, Point3f &joint, double &angle);
#endif