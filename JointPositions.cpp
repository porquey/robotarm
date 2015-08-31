//
//  JointPositions.cpp
//  robotarm
//
//  Created by Forest Fraser on 31/08/15.
//  Copyright (c) 2015 UoA. All rights reserved.
//

#include "JointPositions.h"

void calculateJoints(vector<Point2f>& pixels, Mat cameraMatrix1, Mat cameraMatrix2, Mat distance)
{
    if (pixels.size() != 12){
        cerr << "JointPositions: Invalid number of points" << endl;
    }
    
    vector<Point3f> points;
    
    points.push_back(calculate3DPoint(pixels[0],pixels[1],cameraMatrix1,cameraMatrix2, distance));
    points.push_back(calculate3DPoint(pixels[2],pixels[3],cameraMatrix1,cameraMatrix2, distance));
    points.push_back(calculate3DPoint(pixels[4],pixels[5],cameraMatrix1,cameraMatrix2, distance));
    points.push_back(calculate3DPoint(pixels[6],pixels[7],cameraMatrix1,cameraMatrix2, distance));
    points.push_back(calculate3DPoint(pixels[8],pixels[9],cameraMatrix1,cameraMatrix2, distance));
    points.push_back(calculate3DPoint(pixels[10],pixels[11],cameraMatrix1,cameraMatrix2, distance));

}

Point3f calculate3DPoint(Point2f pixels1, Point2f pixels2, Mat cameraMatrix1, Mat cameraMatrix2, Mat distance){
    
    
    float fx1 = cameraMatrix1.at<double>(0, 0);
    float cx1 = cameraMatrix1.at<double>(0, 2);
    float fy1 = cameraMatrix1.at<double>(1, 1);
    float cy1 = cameraMatrix1.at<double>(1, 2);
    
    float fx2 = cameraMatrix2.at<double>(0, 0);
    float cx2 = cameraMatrix2.at<double>(0, 2);
    float fy2 = cameraMatrix2.at<double>(1, 1);
    float cy2 = cameraMatrix2.at<double>(1, 2);
    
    float xTrans = distance.at<double>(0, 0);
    float yTrans = distance.at<double>(0, 1);
    float zTrans = distance.at<double>(0, 2);
    
    double x1 = pixels1.x - cx1;
    double x2 = pixels2.x - cx2;
    double y1 = pixels1.y - cy1;
    double y2 = pixels2.y - cy2;
    
    double a = (x1)/fx1;
    double b = (x2)/fx2;
    double c = a*b;
    
    double xPos = (c*xTrans + a*zTrans)/(1+c);
    double zPos = (b*xTrans + zTrans)/(1+c);
    double yPos = -((y1)/fy1) * zPos;
    zPos -= zTrans;
    
    Point3f position;
    position.x = xPos;
    position.y = yPos;
    position.z = zPos;
    return position;
}