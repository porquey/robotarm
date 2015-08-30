#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SetBlobColour.h"
#include "BlobHueDetector.h"
#include "CalibrateEnvironment.h"
#include "ControlArm.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define HALF_POINT_X 319.5
#define HALF_POINT_Y 239.5

#define CAMERA1 1
#define CAMERA2 2

using namespace cv;
using namespace std;


float fx1, fy1, cx1, cy1, fx2, fy2, cx2, cy2, xTrans, yTrans, zTrans;

double sum1 = 0, sum2 = 0, sum3 = 0;
int fcount = 0;

void reprojectPoints(double x, double y, double z, Point2f &pt1, Point2f &pt2)
{
    pt1.x = (x*fx1/(z + zTrans)) + cx1;
    pt1.y = (y*fy1/(z + zTrans)) + cy1;
    
    pt2.x = (z*fx2/(x+xTrans))+ cx2;
    pt2.y = (y*fy2/(x+xTrans))+ cy2;
    
}

void drawPoints(Mat &img1, Mat &img2, Point2f pt1, Point2f pt2, Scalar colour)
{
    circle(img1, pt1, 5, colour);//Scalar(r, g, b));
    circle(img2, pt2, 5, colour);//Scalar(r, g, b));
}




int main(int argc, char** argv)
{
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
    
    VideoCapture inputCapture1(CAMERA1);////
    inputCapture1.set(CV_CAP_PROP_FRAME_WIDTH,640);
    inputCapture1.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    VideoCapture inputCapture2(CAMERA2);////
    inputCapture2.set(CV_CAP_PROP_FRAME_WIDTH,640);
    inputCapture2.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    Mat image1, image2;
    
    const string blobFileName = "BlobHSVColour.xml";
    FileStorage bfs(blobFileName, FileStorage::READ);
    
    Mat hsvSize;
    int blobNum = 0;
    
    
    vector<BlobHueDetector> detector;
    BlobHueDetector targetDetector;
    
    bfs["HSV_Size"] >> hsvSize;

    if(hsvSize.data == NULL)
    {
        printf("No BlobHSVColour data\n");
        
        BlobHueDetector blobDetector;
        blobDetector.SetDefaultHSVRanges();
        detector.push_back(blobDetector);
    }
    else
    {
        printf("Found BlobHSVColour data\n");
        blobNum = hsvSize.at<int>(0, 0);
        
        for(int i = 0; i < blobNum; i++)
        {
            string dataName = "HSV_Data_" + to_string(i);
            Mat hsvData;
            bfs[dataName] >> hsvData;
            BlobHueDetector blobDetector;
            HSVRanges ranges;
            
            ranges.lowH = hsvData.at<int>(0, 0);
            ranges.highH = hsvData.at<int>(0, 1);
            ranges.lowS = hsvData.at<int>(1, 0);
            ranges.highS = hsvData.at<int>(1, 1);
            ranges.lowV = hsvData.at<int>(2, 0);
            ranges.highV = hsvData.at<int>(2, 1);
            
            blobDetector.SetHSVRanges(ranges);
            detector.push_back(blobDetector);
            cout << "Set Blob " << i << endl;
        }
        Mat targetData;
        bfs["Target_Data"] >> targetData;
        HSVRanges targetRanges;
        targetRanges.lowH = targetData.at<int>(0, 0);
        targetRanges.highH = targetData.at<int>(0, 1);
        targetRanges.lowS = targetData.at<int>(1, 0);
        targetRanges.highS = targetData.at<int>(1, 1);
        targetRanges.lowV = targetData.at<int>(2, 0);
        targetRanges.highV = targetData.at<int>(2, 1);
        
        targetDetector.SetHSVRanges(targetRanges);

        
    }
    cout << "Initialising" << endl;
    Mat thresh1, thresh2, dst1, dst2;

    clock_t beginTime = clock();;
    //int frames = 0;
    
    ControlArm control;

    
    while(inputCapture1.isOpened() && inputCapture2.isOpened())
    {
        beginTime = clock();
        /*frames++;
        if(float(clock() - beginTime)/CLOCKS_PER_SEC >= 1)
        {
            //cout << "FPS: " << frames << " " << float(clock() - beginTime)/CLOCKS_PER_SEC <<endl;
            frames = 0;
            beginTime = clock();
        }*/
        
        // Read and transform images from cameras
        inputCapture1.read(image1);
        inputCapture2.read(image2);
        Mat t1 = image1.clone();
        Mat t2 = image2.clone();
        remap(t1, image1, mapX1, mapY1, INTER_LINEAR);
        remap(t2, image2, mapX2, mapY2, INTER_LINEAR);
        t1.release();
        t2.release();
        
        //cout << "Image remapping: " << float(clock() - beginTime)/CLOCKS_PER_SEC << endl;
        sum1 += float(clock() - beginTime)/CLOCKS_PER_SEC;
        
        beginTime = clock();
        
        vector<bool> detectedVec;
        vector<KeyPoint> keypointVec1, keypointVec2;
        vector<Point3D> coords, coordsLast;
        
        for(int i = 0; i < blobNum; i++)
        {
            KeyPoint keypoint1, keypoint2;
            bool detected = detector[i].GetBlobCentres(image1, image2, keypoint1, keypoint2);

            double xPos = 0, yPos = 0, zPos = 0;
            double x1, x2, y1, y2, a, b, c;

           
            x1 = keypoint1.pt.x - cx1;
            x2 = keypoint2.pt.x - cx2;
            y1 = keypoint1.pt.y - cy1;
            y2 = keypoint2.pt.y - cy2;
            
            a = (x1)/fx1;
            b = (x2)/fx2;
            c = a*b;
            
            xPos = (c*xTrans + a*zTrans)/(1+c);
            zPos = (b*xTrans + zTrans)/(1+c);
            yPos = -((y1)/fy1) * zPos;
            zPos -= zTrans;
            
            Point3D coordTemp;
            coordTemp.x = xPos;
            coordTemp.y = yPos;
            coordTemp.z = zPos;
            coords.push_back(coordTemp);
            detectedVec.push_back(detected);
            keypointVec1.push_back(keypoint1);
            keypointVec2.push_back(keypoint2);
            
        }
        
        KeyPoint keypoint1, keypoint2;
        targetDetector.GetBlobCentres(image1, image2, keypoint1, keypoint2);
        
        double xPos = 0, yPos = 0, zPos = 0;
        double x1, x2, y1, y2, a, b, c;
        
        x1 = keypoint1.pt.x - cx1;
        x2 = keypoint2.pt.x - cx2;
        y1 = keypoint1.pt.y - cy1;
        y2 = keypoint2.pt.y - cy2;
        
        a = (x1)/fx1;
        b = (x2)/fx2;
        c = a*b;
        
        xPos = (c*xTrans + a*zTrans)/(1+c);
        zPos = (b*xTrans + zTrans)/(1+c);
        yPos = -((y1)/fy1) * zPos;
        zPos -= zTrans;
        
        Point3D targetCoord;
        targetCoord.x = xPos;
        targetCoord.y = yPos;
        targetCoord.z = zPos;
        keypointVec1.push_back(keypoint1);
        keypointVec2.push_back(keypoint2);
        

        
        //cout << "Blob detection: " << float(clock() - beginTime)/CLOCKS_PER_SEC << endl;
        sum2 += float(clock() - beginTime)/CLOCKS_PER_SEC;

        beginTime = clock();
        
        
        cv::drawKeypoints(image1, keypointVec1, dst1, cv::Scalar(0,255,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawKeypoints(image2, keypointVec2, dst2, cv::Scalar(0,255,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        for(int i = 0; i < blobNum; i++)
        {
            string posStr;
            if(!detectedVec[i])
            {
                posStr = "X: " + to_string(0.00000000) + "  Y: " + to_string(0.00000000) + "  Z: " + to_string(0.00000000);
            }
            else
            {
                posStr = "X: " + to_string(coords[i].x) + "  Y: " + to_string(coords[i].y) + "  Z: " + to_string(coords[i].z);
            }
            
            putText(dst1, posStr, Point(5, 15 * (i + 1)), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
            
        }
        
        string posStr = "X: " + to_string(targetCoord.x) + "  Y: " + to_string(targetCoord.y) + "  Z: " + to_string(targetCoord.z);

        putText(dst1, posStr, Point(5, 15 * 4), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));

        /*
        Vector3D vecA, vecB;
        double jointAngle = 0;
        
        //cout << coords[0].x << " " << coords[0].y << " " << coords[0].z << endl;
        calculateVector(coords[0], coords[1], vecA);
        calculateVector(coords[1], coords[2], vecB);

        jointAngle = calculateAngle(vecA, vecB);
        
        string angleStr = "Angle: " + to_string(jointAngle);
        putText(dst2, angleStr, Point(5, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        */
        /*
        Point2f point1;
        Point2f point2;
        
        double yOffset = 0;
        
        for(int i = 0; i < 5; i++)
        {
            for(int j = 0; j < 1; j++)
            {
                reprojectPoints(-200 + i * 100, yOffset, -200 + j * 100, point1, point2);
                Scalar pixel = Scalar(255 - i * 50, 255 - j * 50, 255);
                drawPoints(dst1, dst2, point1, point2, pixel);
            }
        }
        */
        
        
        
        
        
        
        
        imshow("Camera2", dst2);
        imshow("Camera1", dst1);
                
        Point3D base = coords[0];
        
        base.y -= 100;
        
        coords.insert(coords.begin(), base);
        control.SetArmPose(coords);
        control.CalculateLinkLengths();
        
        double currAngles[3];
        control.GetCurrentPose(currAngles);

        control.SetTarget(targetCoord);
        
        double angles[3];
        control.GetArmPose(angles);
        
        
        sum3 += float(clock() - beginTime)/CLOCKS_PER_SEC;
        beginTime = clock();
        
        char ch = waitKey(15);
        if(ch == 'e')
        {
            CalibrateEnvironment(inputCapture1, inputCapture2);
        }
        else if(ch == 'c')
        {
            destroyAllWindows();
            SetBlobColour(inputCapture1, inputCapture2);
            cout << "Setting Blob HSV Data" << endl;
            FileStorage bfs(blobFileName, FileStorage::READ);
            
            bfs["HSV_Size"] >> hsvSize;
            detector.clear();
            if(hsvSize.data == NULL)
            {
                printf("No BlobHSVColour data\n");
                
                BlobHueDetector blobDetector;
                blobDetector.SetDefaultHSVRanges();
                detector.push_back(blobDetector);
            }
            else
            {
                printf("Found BlobHSVColour data\n");
                blobNum = hsvSize.at<int>(0, 0);
                
                for(int i = 0; i < blobNum; i++)
                {
                    string dataName = "HSV_Data_" + to_string(i);
                    Mat hsvData;
                    bfs[dataName] >> hsvData;
                    BlobHueDetector blobDetector;
                    HSVRanges ranges;
                    
                    ranges.lowH = hsvData.at<int>(0, 0);
                    ranges.highH = hsvData.at<int>(0, 1);
                    ranges.lowS = hsvData.at<int>(1, 0);
                    ranges.highS = hsvData.at<int>(1, 1);
                    ranges.lowV = hsvData.at<int>(2, 0);
                    ranges.highV = hsvData.at<int>(2, 1);
                    
                    blobDetector.SetHSVRanges(ranges);
                    detector.push_back(blobDetector);
                }
                
                Mat targetData;
                bfs["Target_Data"] >> targetData;
                HSVRanges targetRanges;
                targetRanges.lowH = targetData.at<int>(0, 0);
                targetRanges.highH = targetData.at<int>(0, 1);
                targetRanges.lowS = targetData.at<int>(1, 0);
                targetRanges.highS = targetData.at<int>(1, 1);
                targetRanges.lowV = targetData.at<int>(2, 0);
                targetRanges.highV = targetData.at<int>(2, 1);
                
                targetDetector.SetHSVRanges(targetRanges);
                
            }
            destroyAllWindows();
        }
        /*else if(ch == 'p')
        {
            for(int i = 0; i < blobNum; i++)
            {
                cout << "Blob " << i << ":" << endl;
                if(detectedVec[i])
                {
                    cout << "center1: " << keypointVec1[i].pt << endl;
                    cout << "center2: " << keypointVec2[i].pt << endl;
                    
                    cout << "X: " << coords[i].x << endl;
                    cout << "Y: " << coords[i].y << endl;
                    cout << "Z: " << coords[i].z << endl;
                    
                    cout << "X movement: " << (coords[i].x - coordsLast[i].x) << endl;
                    cout << "Y movement: " << (coords[i].y - coordsLast[i].y) << endl;
                    cout << "Z movement: " << (coords[i].z - coordsLast[i].z) << endl;
                    
                    coordsLast[i].x = coords[i].x;
                    coordsLast[i].y = coords[i].y;
                    coordsLast[i].z = coords[i].z;
                }
            }
        }*/
        //cout << "Draw on image: " << float(clock() - beginTime)/CLOCKS_PER_SEC << endl;
        
        fcount++;
        //cout << fcount << endl;
        if(fcount == 100)
        {
            cout << "REMAP average: " << sum1/100 << endl;
            cout << "DETECT average: " << sum2/100 << endl;
            cout << "DRAW average: " << sum3/100 << endl;
            fcount = 0;
            sum1 = sum2 = sum3 = 0;
            //break;
        }
        
        /*else if(ch == 'b')
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
            
        }*/
        
    }
    
}
