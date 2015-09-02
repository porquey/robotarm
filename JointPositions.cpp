//
//  JointPositions.cpp
//  robotarm
//
//  Created by Forest Fraser on 31/08/15.
//  Copyright (c) 2015 UoA. All rights reserved.
//

#include "JointPositions.h"

bool DetermineBasePairs(vector<KeyPoint> &random, vector<KeyPoint> &sorted)
{
    if(random.size() != 4)
    {
        return false;
    }
    
    KeyPoint a = random[0];
    KeyPoint b = random[1];
    KeyPoint c = random[2];
    KeyPoint d = random[3];
    
    sorted.clear();
    
    if(a.pt.y > b.pt.y)
    {
        sorted.push_back(a);
        sorted.push_back(b);
    }
    else
    {
        sorted.push_back(b);
        sorted.push_back(a);
    }
    if(c.pt.y > d.pt.y)
    {
        sorted.push_back(c);
        sorted.push_back(d);
    }
    else
    {
        sorted.push_back(d);
        sorted.push_back(c);
    }
    return true;
}


bool DeterminePairs(vector<KeyPoint> &random, vector<KeyPoint> &sorted, Point pt1, Point pt2)
{
    if(random.size() != 4)
    {
        return false;
    }
    
    KeyPoint a = random[0];
    KeyPoint b = random[1];
    KeyPoint c = random[2];
    KeyPoint d = random[3];
    
    sorted.clear();
    
    if(CalculateDisplacement(a.pt, pt1) < CalculateDisplacement(b.pt, pt1))
    {
        sorted.push_back(a);
        sorted.push_back(b);
    }
    else
    {
        sorted.push_back(b);
        sorted.push_back(a);
    }
    if(CalculateDisplacement(c.pt, pt1) < CalculateDisplacement(d.pt, pt1))
    {
        sorted.push_back(c);
        sorted.push_back(d);
    }
    else
    {
        sorted.push_back(d);
        sorted.push_back(c);
    }
    return true;
}

void CalculateLinkVector(vector<Point2f> points, vector<Point3f> linkPoints, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation)
{
    linkPoints.push_back(Calculate3DPoint(points[0], points[2], cameraMatrix1, cameraMatrix2, translation));
    linkPoints.push_back(Calculate3DPoint(points[1], points[3], cameraMatrix1, cameraMatrix2, translation));
}

void CalculateJoint(vector<Point3f> link1, vector<Point3f> link2, Point3f& joint, double &angle)
{
    Point3f vec1 = CalculateVector(link1[0], link1[1]);
    Point3f vec2 = CalculateVector(link2[1], link2[0]);
    angle = CalculateAngle(vec1, vec2);
    if(angle == 0)
    {
        joint = link1[0];
        return;
    }
    
    double v1v2 = CalculateDotProduct(vec1, vec2);
    double v1v1 = CalculateDotProduct(vec1, vec1);
    double v2v2 = CalculateDotProduct(vec2, vec2);
    double p1v2 = CalculateDotProduct(link1[0], vec2);
    double p2v2 = CalculateDotProduct(link2[1], vec2);
    double p1v1 = CalculateDotProduct(link1[0], vec1);
    double p2v1 = CalculateDotProduct(link2[1], vec1);

    
    double s = (v1v1 * (p1v2 - p2v2) + v1v2 * (p2v1 - p1v1)) / (v2v2 * v1v1 - v1v2 * v1v2);
    double t = (p2v1 - p1v1 + s * v1v2) / v1v1;
    Point3f pt1 = link1[0] + t * vec1;
    Point3f pt2 = link2[1] + s * vec2;
    //cout << "S: " << s << " T: " << t << endl;
    //cout << "POINT1: " << pt1 << " POINT2: " << pt2 << endl;
    joint = (pt1 + pt2) * 0.5;
    //cout << "JOINT: " << joint << endl;
    //Point3f distVec = s * vec2 + link2[1] - t * vec1 - link1[0];
    //cout << "DIST VEC: " << distVec << endl;
    //cout << " DIST: " << CalculateLength(distVec) << endl;
}

Point3f Calculate3DPoint(Point2f pt1, Point2f pt2, Mat &cameraMatrix1, Mat &cameraMatrix2, Mat &translation){
    
    double fx1 = cameraMatrix1.at<double>(0, 0);
    double cx1 = cameraMatrix1.at<double>(0, 2);
    double fy1 = cameraMatrix1.at<double>(1, 1);
    double cy1 = cameraMatrix1.at<double>(1, 2);
    
    double fx2 = cameraMatrix2.at<double>(0, 0);
    double cx2 = cameraMatrix2.at<double>(0, 2);
    //double fy2 = cameraMatrix2.at<double>(1, 1);
    //double cy2 = cameraMatrix2.at<double>(1, 2);
    
    double xTrans = translation.at<double>(0, 0);
    //double yTrans = distance.at<double>(0, 1);
    double zTrans = translation.at<double>(0, 2);
    
    double x1 = pt1.x - cx1;
    double x2 = pt2.x - cx2;
    double y1 = pt1.y - cy1;
    //double y2 = pixels2.y - cy2;
    
    double a = (x1)/fx1;
    double b = (x2)/fx2;
    
    double xPos = (b*xTrans + zTrans)/(1/a+b);
    double zPos = (xTrans - a*zTrans)/(1/b+a);
    double yPos = -((y1)/fy1) * (zPos+zTrans);

    return Point3f(xPos, yPos, zPos);
}

double CalculateDisplacement(Point a, Point b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double CalculateAngle(Point3f a, Point3f b)
{
    double mag1 = CalculateLength(a);
    double mag2 = CalculateLength(b);
    
    if(mag1 == 0 || mag2 == 0)
    {
        return 0;
    }
    double angle = acos((a.x * b.x + a.y * b.y + a.z * b.z)/(mag1 * mag2));
    
    return angle;
}

Point3f CalculateVector(Point3f a, Point3f b)
{
    return a - b;
}

double CalculateDotProduct(Point3f a, Point3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double CalculateLength(Point3f a)
{
    return sqrt(CalculateDotProduct(a, a));
}