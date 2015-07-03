#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "BlobHueDetector.h"
#include "CalibrateEnvironment.h"
#include "SetBlobColour.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define HALF_POINT_X 319.5
#define HALF_POINT_Y 239.5


using namespace cv;
using namespace std;

float fx1, fy1, cx1, cy1, fx2, fy2, cx2, cy2, xTrans, yTrans, zTrans;

void reprojectPoints(double x, double y, double z, Point2f &pt1, Point2f &pt2)
{
    pt1.x = (x*fx1/(z + zTrans)) + cx1;
    pt1.y = (y*fy1/(z + zTrans)) + cy1;
    
    pt2.x = (z*fx2/(x+xTrans))+ cx2;
    pt2.y = (y*fy2/(x+xTrans))+ cy2;

}

void drawPoints(Mat &img1, Mat &img2, Point2f pt1, Point2f pt2, Scalar colour)
{
    static int r = 255;
    static int g = 0;
    static int b = 0;
    static int sel = 0;
    circle(img1, pt1, 5, colour);//Scalar(r, g, b));
    circle(img2, pt2, 5, colour);//Scalar(r, g, b));
    /*if(sel == 0)
    {
        g = r;
        r = 0;
        sel = 1;
    }
    else if(sel == 1)
    {
        b = g;
        g = 0;
        sel = 2;
    }
    else
    {
        r = b - 32;
        if(r <= 31)
        {
            r = 255;
        }
        b = 0;
        sel = 0;
    }*/
}


int main(int argc, char** argv)
{
    float xPos = 0, yPos = 0, zPos = 0, xPosLast = 0, yPosLast = 0, zPosLast = 0;

    BlobHueDetector detector;

    
    Mat cameraMatrix1, cameraMatrix2, mapX1, mapY1, mapX2, mapY2, translation;
    
    
    const string calibFileName = "EnvironmentCalibration.xml";
    
    cout << "Attempting to open configuration files" << endl;
    FileStorage fs(calibFileName, FileStorage::READ);
    
    fs["Camera_Matrix_1"] >> cameraMatrix1;
    fs["Camera_Matrix_2"] >> cameraMatrix2;
    fs["Mapping_X_1"] >> mapX1;
    fs["Mapping_Y_1"] >> mapY1;
    fs["Mapping_X_2"] >> mapX2;
    fs["Mapping_Y_2"] >> mapY2;
    fs["Translation"] >> translation;
    
    if (cameraMatrix1.data == NULL ||
        cameraMatrix2.data == NULL)
    {
        printf("Could not load camera intrinsics\n");
    }
    
    fx1 = cameraMatrix1.at<double>(0, 0);
    cx1 = cameraMatrix1.at<double>(0, 2);
    fy1 = cameraMatrix1.at<double>(1, 1);
    cy1 = cameraMatrix1.at<double>(1, 2);
    
    fx2 = cameraMatrix2.at<double>(0, 0);
    cx2 = cameraMatrix2.at<double>(0, 2);
    fy2 = cameraMatrix2.at<double>(1, 1);
    cy2 = cameraMatrix2.at<double>(1, 2);
    
    xTrans = translation.at<double>(0,0);
    yTrans = translation.at<double>(1, 0);
    zTrans = translation.at<double>(2, 0);
    
    VideoCapture inputCapture1(1);////
    inputCapture1.set(CV_CAP_PROP_FRAME_WIDTH,640);
    inputCapture1.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    VideoCapture inputCapture2(2);////
    inputCapture2.set(CV_CAP_PROP_FRAME_WIDTH,640);
    inputCapture2.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    Mat image1, image2;
    
    const string blobFileName = "BlobHSVColour.xml";
    FileStorage bfs(blobFileName, FileStorage::READ);
    Mat hsvData;
    
    bfs["HSV_Data"] >> hsvData;
    if(hsvData.data == NULL)
    {
        printf("No BlobHSVColour data\n");
        detector.setHSVRanges(0, 179, 0, 255, 0, 255);
    }
    else
    {
        cout << hsvData.at<int>(0, 0) << " " << hsvData.at<int>(0, 1) << " " << hsvData.at<int>(1, 0) << " " << hsvData.at<int>(1, 1) << " " << hsvData.at<int>(2, 0) << " " << hsvData.at<int>(2, 1) << endl;

        detector.setHSVRanges(static_cast<int>(hsvData.at<int>(0, 0)), static_cast<int>(hsvData.at<int>(0, 1)), static_cast<int>(hsvData.at<int>(1, 0)), static_cast<int>(hsvData.at<int>(1, 1)), static_cast<int>(hsvData.at<int>(2, 0)), static_cast<int>(hsvData.at<int>(2, 1)));
    }
    
    string posStr = "X: " + to_string(xPos) + "\tY: " + to_string(yPos) + "\tZ: " + to_string(zPos);
    
    Point2f center1, center2;
    Mat thresh1, thresh2, dst1, dst2;
    double area1, area2, conv1, conv2;
    double x1, x2, y1, y2, a, b, c;
    
    bool showThresh = false;
    
    while(inputCapture1.isOpened() && inputCapture2.isOpened())
    {
        
        
        // Read and transform images from cameras
        inputCapture1.read(image1);
        inputCapture2.read(image2);
        Mat t1 = image1.clone();
        Mat t2 = image2.clone();
        remap(t1, image1, mapX1, mapY1, INTER_LINEAR);
        remap(t2, image2, mapX2, mapY2, INTER_LINEAR);
        t1.release();
        t2.release();
        
        
        bool detected1 = detector.getBlobCenter(image1, thresh1, center1, dst1, "Cam1", area1, conv1);
        bool detected2 = detector.getBlobCenter(image2, thresh2, center2, dst2, "Cam2", area2, conv2);
        
        
        x1 = center1.x - cx1;
        x2 = center2.x - cx2;
        y1 = center1.y - cy1;
        y2 = center2.y - cy2;
        
        
        a = (x1)/fx1;
        b = (x2)/fx2;
        c = a*b;
        
        xPos = (c*xTrans + a*zTrans)/(1+c);
        zPos = (b*xTrans + zTrans)/(1+c);
        yPos = -((y1)/fy1) * zPos;
        zPos -= zTrans;
        
        posStr = "X: " + to_string(xPos) + "  Y: " + to_string(yPos) + "  Z: " + to_string(zPos);
        
        putText(dst1, posStr, Point(5,15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        
        Point2f point1;
        Point2f point2;
        reprojectPoints(0, -68, 0, point1, point2);
        drawPoints(dst1, dst2, point1, point2, Scalar(255, 0, 0));
        reprojectPoints(100, -68, 100, point1, point2);
        drawPoints(dst1, dst2, point1, point2, Scalar(0, 255, 0));
        reprojectPoints(-100, -68, -100, point1, point2);
        drawPoints(dst1, dst2, point1, point2, Scalar(0, 0, 255));
        
        imshow("Camera2", dst2);
        imshow("Camera1", dst1);
        
        if(showThresh)
        {
            imshow("Thresh1", thresh1);
            imshow("Thresh2", thresh2);
        }
        
        char ch = waitKey(15);
        if(ch == 'e')
        {
            CalibrateEnvironment(inputCapture1, inputCapture2);
        }
        else if(ch == 'c')
        {
            destroyAllWindows();
            SetBlobColour(inputCapture1);
            cout << "nop " << endl;
            FileStorage bfs(blobFileName, FileStorage::READ);
            
            bfs["HSV_Data"] >> hsvData;
            cout << hsvData.at<int>(0, 0) << " " << hsvData.at<int>(0, 1) << " " << hsvData.at<int>(1, 0) << " " << hsvData.at<int>(1, 1) << " " << hsvData.at<int>(2, 0) << " " << hsvData.at<int>(2, 1) << endl;
            detector.setHSVRanges(static_cast<int>(hsvData.at<int>(0, 0)), static_cast<int>(hsvData.at<int>(0, 1)), static_cast<int>(hsvData.at<int>(1, 0)), static_cast<int>(hsvData.at<int>(1, 1)), static_cast<int>(hsvData.at<int>(2, 0)), static_cast<int>(hsvData.at<int>(2, 1)));
            destroyAllWindows();
        }
        else if(ch == 't')
        {
            if(showThresh)
            {
                showThresh = false;
                destroyAllWindows();
            }
            else
            {
                showThresh = true;
            }
        }
        else if(ch == 'p')
        {
            
            //cout << "center x1: " << center1.x << endl;
            cout << "center y1: " << center1.y << endl;
            //cout << "center x2: " << center2.x << endl;
            cout << "center y2: " << center2.y << endl;
            
            
            //cout << "x1: " << x1 << endl;
            cout << "y1: " << y1 << endl;
            cout << "cy: " << cy1 << endl;
            //cout << "x2: " << x2 << endl;
            cout << "y2: " << y2 << endl;
            
            cout << "X: " << xPos << endl;
            cout << "Y: " << yPos << endl;
            cout << "Z: " << zPos << endl;
            
            cout << "X movement: " << (xPos - xPosLast) << endl;
            cout << "Y movement: " << (yPos - yPosLast) << endl;
            cout << "Z movement: " << (zPos - zPosLast) << endl;
            
            xPosLast = xPos;
            yPosLast = yPos;
            zPosLast = zPos;
        }
        else if(ch == 'b')
        {
            if(detected1)
            {
                cout << "Cam1 : blob area: " << area1 << " convexity: " << conv1 << endl;
            }
            else
            {
                cout << "Cam1 : no blob" << endl;
            }
            
            if(detected2)
            {
                cout << "Cam2 : blob area: " << area2 << " convexity: " << conv2 << endl;
            }
            else
            {
                cout << "Cam2 : no blob" << endl;
            }
            
        }
    }
    
}
