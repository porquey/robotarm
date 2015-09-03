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
#include "JointPositions.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define HALF_POINT_X 319.5
#define HALF_POINT_Y 239.5

#define CAMERA1 1
#define CAMERA2 2

#define LINK0 15.8
#define LINK1 20.2
#define LINK2 17.4


using namespace cv;
using namespace std;


float fx1, fy1, cx1, cy1, fx2, fy2, cx2, cy2, xTrans, yTrans, zTrans;

double sum1 = 0, sum2 = 0, sum3 = 0;
int fcount = 0;

int reprojectVal = 0, height = 0;

bool checkBlobs = false;




void drawPoints(Mat &img1, Mat &img2, Point2f pt1, Point2f pt2, Scalar colour)
{
    circle(img1, pt1, 5, colour);//Scalar(r, g, b));
    circle(img2, pt2, 5, colour);//Scalar(r, g, b));
}

int main(int argc, char** argv)
{
    Mat cameraMatrix1, cameraMatrix2, mapX1, mapY1, mapX2, mapY2, translation;
    
    const string calibFileName = "EnvironmentCalibration.xml";
    
    cerr << "Attempting to open configuration files" << endl;
    FileStorage fs(calibFileName, FileStorage::READ);
    
    fs["Camera_Matrix_1"] >> cameraMatrix1;
    fs["Camera_Matrix_2"] >> cameraMatrix2;
    fs["Mapping_X_1"] >> mapX1;
    fs["Mapping_Y_1"] >> mapY1;
    fs["Mapping_X_2"] >> mapX2;
    fs["Mapping_Y_2"] >> mapY2;
    fs["Translation"] >> translation;
    
    if (cameraMatrix1.data == NULL || cameraMatrix2.data == NULL)
    {
        cerr << "Could not load camera intrinsics" << endl;
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
        cerr << "No BlobHSVColour data" << endl;
        
        BlobHueDetector blobDetector;
        blobDetector.SetDefaultHSVRanges();
        detector.push_back(blobDetector);
    }
    else
    {
        cerr << "Found BlobHSVColour data" << endl;
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
            cerr << "Set Blob " << i << endl;
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
    
    cerr << "Initialising" << endl;
    Mat thresh1, thresh2, dst1, dst2;

    clock_t beginTime = clock();;
    //int frames = 0;
    ControlArm control(LINK0, LINK1, LINK2);

    while(inputCapture1.isOpened() && inputCapture2.isOpened())
    {
        beginTime = clock();
        /*frames++;
        if(float(clock() - beginTime)/CLOCKS_PER_SEC >= 1)
        {
            //cerr << "FPS: " << frames << " " << float(clock() - beginTime)/CLOCKS_PER_SEC <<endl;
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
        
        //cerr << "Image remapping: " << float(clock() - beginTime)/CLOCKS_PER_SEC << endl;
        sum1 += float(clock() - beginTime)/CLOCKS_PER_SEC;
        beginTime = clock();
        
        vector<bool> detectedVec;
        vector<Point3f> coords, coordsLast;
        static Point3f base, tip;
        
        static Point3f joint1, joint2;
        double angle1, angle2;
        static Point3f begin0, begin1, begin2, end0, end1, end2;
        static Point3f link0[2], link1[2], link2[2];
        static KeyPoint random0[4], sorted0[4], random1[4], sorted1[4], random2[4], sorted2[4];

        
        if(checkBlobs)
        {
            vector<KeyPoint> keypointVec1, keypointVec2;
            
            for(int i = 0; i < blobNum; i++)
            {
                KeyPoint keypoint1, keypoint2;
                bool detected = detector[i].GetBlobCentres(image1, image2, keypoint1, keypoint2);
                
                Point3f coordTemp = Calculate3DPoint(keypoint1.pt, keypoint2.pt, cameraMatrix1, cameraMatrix2, translation);
                coords.push_back(coordTemp);
                detectedVec.push_back(detected);
                keypointVec1.push_back(keypoint1);
                keypointVec2.push_back(keypoint2);
                
            }
            
            cv::drawKeypoints(image1, keypointVec1, dst1, cv::Scalar(0, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(image2, keypointVec2, dst2, cv::Scalar(0, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        }
        else
        {
            vector<KeyPoint> beginVec1, beginVec2, endVec1, endVec2;
            /*for(int i = 0; i < blobNum; i++)
            {
                KeyPoint p1, p2, p3, p4;
                bool detected = detector[i].GetStripVectors(image1, image2, p1, p2, p3, p4);
                
                if(detected)
                {
                    vector<KeyPoint> random, sorted;
                    random.push_back(p1);
                    random.push_back(p2);
                    random.push_back(p3);
                    random.push_back(p4);
                    
                    DetermineBasePairs(random, sorted);
                    
                    beginVec1.push_back(sorted[0]);
                    endVec1.push_back(sorted[1]);
                    beginVec2.push_back(sorted[2]);
                    endVec2.push_back(sorted[3]);
                }
                
            }*/
            KeyPoint p1, p2, p3, p4;
            
            dst1 = image1.clone();
            dst2 = image2.clone();
            
            static bool firstDetect0 = false, firstDetect1 = false;

            bool detected0 = detector[0].GetStripVectors(image1, image2, p1, p2, p3, p4);
            if(detected0)
            {
                if(!firstDetect0)
                {
                    firstDetect0 = true;
                }
                random0[0] = p1;
                random0[1] = p2;
                random0[2] = p3;
                random0[3] = p4;
                //cerr << "Detected base!" << endl;
                DetermineBasePairs(random0, sorted0);
                begin0 = Calculate3DPoint(sorted0[0].pt, sorted0[2].pt, cameraMatrix1, cameraMatrix2, translation);
                end0 = Calculate3DPoint(sorted0[1].pt, sorted0[3].pt, cameraMatrix1, cameraMatrix2, translation);
                link0[0] = begin0;
                link0[1] = end0;
                base = begin0;
            }
                
            bool detected1 = detector[1].GetStripVectors(image1, image2, p1, p2, p3, p4);
            if(detected1 && firstDetect0)
            {
                if(!firstDetect1)
                {
                    firstDetect1 = true;
                }
                random1[0] = p1;
                random1[1] = p2;
                random1[2] = p3;
                random1[3] = p4;
                DeterminePairs(random1, sorted1, sorted0[0].pt, sorted0[2].pt);
                begin1 = Calculate3DPoint(sorted1[0].pt, sorted1[2].pt, cameraMatrix1, cameraMatrix2, translation);
                end1 = Calculate3DPoint(sorted1[1].pt, sorted1[3].pt, cameraMatrix1, cameraMatrix2, translation);
                link1[0] = begin1;
                link1[1] = end1;
                
                CalculateJoint(link0, link1, joint1, angle1);
            }
                    //cerr << "Detected joint 1! Angle is: " << angle1 << endl;

            bool detected2 = detector[2].GetStripVectors(image1, image2, p1, p2, p3, p4);
            if(detected2 && firstDetect1)
            {
                random2[0] = p1;
                random2[1] = p2;
                random2[2] = p3;
                random2[3] = p4;
                DeterminePairs(random2, sorted2, sorted1[0].pt, sorted1[2].pt);
                
                begin2 = Calculate3DPoint(sorted2[0].pt, sorted2[2].pt, cameraMatrix1, cameraMatrix2, translation);
                end2 = Calculate3DPoint(sorted2[1].pt, sorted2[3].pt, cameraMatrix1, cameraMatrix2, translation);
                
                link2[0] = begin2;
                link2[1] = end2;
                CalculateJoint(link1, link2, joint2, angle2);
                //cerr << "Detected joint 2! Angle is: " << angle2 << endl;
                tip = begin2;
            }
            
            Point2f joint11, joint12, joint21, joint22;
            Point2f begin01, begin02, end01, end02, begin11, begin12, end11, end12, begin21, begin22, end21, end22;
            

            ReprojectPoints(begin0, begin01, begin02, cameraMatrix1, cameraMatrix2, translation);
            ReprojectPoints(begin1, begin11, begin12, cameraMatrix1, cameraMatrix2, translation);
            ReprojectPoints(end0, end01, end02, cameraMatrix1, cameraMatrix2, translation);
            ReprojectPoints(end1, end11, end12, cameraMatrix1, cameraMatrix2, translation);
            ReprojectPoints(begin2, begin21, begin22, cameraMatrix1, cameraMatrix2, translation);
            ReprojectPoints(end2, end21, end22, cameraMatrix1, cameraMatrix2, translation);
            
            ReprojectPoints(joint1, joint11, joint12, cameraMatrix1, cameraMatrix2, translation);
            ReprojectPoints(joint2, joint21, joint22, cameraMatrix1, cameraMatrix2, translation);

            circle(dst1, begin01, 5, Scalar(0, 0, 0));
            circle(dst1, begin11, 5, Scalar(0, 0, 0));
            circle(dst1, begin21, 5, Scalar(0, 0, 0));
            circle(dst2, begin02, 5, Scalar(0, 0, 0));
            circle(dst2, begin12, 5, Scalar(0, 0, 0));
            circle(dst2, begin22, 5, Scalar(0, 0, 0));

            line(dst1, begin01, end01, Scalar(0, 0, 0));
            line(dst1, begin11, end11, Scalar(0, 0, 0));
            line(dst1, begin21, end21, Scalar(0, 0, 0));
            
            line(dst2, begin02, end02, Scalar(0, 0, 0));
            line(dst2, begin12, end12, Scalar(0, 0, 0));
            line(dst2, begin22, end22, Scalar(0, 0, 0));

            
            circle(dst1, joint11, 5, Scalar(0, 255, 0));
            circle(dst1, joint21, 5, Scalar(0, 255, 0));
            circle(dst2, joint12, 5, Scalar(0, 255, 0));
            circle(dst2, joint22, 5, Scalar(0, 255, 0));

            
            circle(dst1, sorted0[0].pt, 5, Scalar(255, 0, 0));
            circle(dst1, sorted0[1].pt, 5, Scalar(0, 0, 255));
            line(dst1, sorted0[0].pt, sorted0[1].pt, Scalar(255,0,0));
            
            circle(dst2, sorted0[2].pt, 5, Scalar(255, 0, 0));
            circle(dst2, sorted0[3].pt, 5, Scalar(0, 0, 255));
            line(dst2, sorted0[2].pt, sorted0[3].pt, Scalar(255,0,0));
            
            
            
            circle(dst1, sorted1[0].pt, 5, Scalar(255, 0, 0));
            circle(dst1, sorted1[1].pt, 5, Scalar(0, 0, 255));
            line(dst1, sorted1[0].pt, sorted1[1].pt, Scalar(255,0,0));

            circle(dst2, sorted1[2].pt, 5, Scalar(255, 0, 0));
            circle(dst2, sorted1[3].pt, 5, Scalar(0, 0, 255));
            line(dst2, sorted1[2].pt, sorted1[3].pt, Scalar(255,0,0));
            
            
            
            circle(dst1, sorted2[0].pt, 5, Scalar(255, 0, 0));
            circle(dst1, sorted2[1].pt, 5, Scalar(0, 0, 255));
            line(dst1, sorted2[0].pt, sorted2[1].pt, Scalar(255,0,0));
                
            circle(dst2, sorted2[2].pt, 5, Scalar(255, 0, 0));
            circle(dst2, sorted2[3].pt, 5, Scalar(0, 0, 255));
            line(dst2, sorted2[2].pt, sorted2[3].pt, Scalar(255,0,0));
            
            string posStr = "X: " + to_string(joint1.x) + "  Y: " + to_string(joint1.y) + "  Z: " + to_string(joint1.z);
            putText(dst1, posStr, Point(5, 15 * 1), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
            posStr = "X: " + to_string(joint2.x) + "  Y: " + to_string(joint2.y) + "  Z: " + to_string(joint2.z);
            putText(dst1, posStr, Point(5, 15 * 2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
            
            string angleStr = "JOINT1: " + to_string(angle1);
            putText(dst2, angleStr, Point(5, 15 * 2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
            angleStr = "JOINT2: " + to_string(angle2);
            putText(dst2, angleStr, Point(5, 15 * 3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));

            /*cv::drawKeypoints(image1, beginVec1, dst1, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(image2, beginVec2, dst2, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(dst1, endVec1, dst1, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(dst2, endVec2, dst2, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);*/
        }
        
        KeyPoint keypoint1, keypoint2;
        targetDetector.GetBlobCentres(image1, image2, keypoint1, keypoint2);
        
        Point3f targetCoord = Calculate3DPoint(keypoint1.pt, keypoint2.pt, cameraMatrix1, cameraMatrix2, translation);
        
        
        circle(dst1, keypoint1.pt, 5, Scalar(0, 255, 255));
        circle(dst2, keypoint2.pt, 5, Scalar(0, 255, 255));
        
        /*vector<KeyPoint> targetVec1;
        vector<KeyPoint> targetVec2;
        
        targetVec1.push_back(keypoint1);
        targetVec2.push_back(keypoint2);
        
        cv::drawKeypoints(dst1, targetVec1, dst1, cv::Scalar(255,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawKeypoints(dst2, targetVec2, dst2, cv::Scalar(255,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        */
        
        //cerr << "Blob detection: " << float(clock() - beginTime)/CLOCKS_PER_SEC << endl;
        sum2 += float(clock() - beginTime)/CLOCKS_PER_SEC;
        beginTime = clock();
        
        
        /*for(int i = 0; i < detectedVec.size(); i++)
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
        }*/
        
        string posStr = "X: " + to_string(targetCoord.x) + "  Y: " + to_string(targetCoord.y) + "  Z: " + to_string(targetCoord.z);
        putText(dst1, posStr, Point(5, 15 * 4), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        
        
        /////CONTROL/////
        
        
        vector<Point3f> joints;
        joints.push_back(base);
        joints.push_back(joint1);
        joints.push_back(joint2);
        joints.push_back(tip);
        //cerr << base << " " << joint1 << " " << joint2 << " " << tip << endl;
        control.SetArmPose(joints);
        control.SetTarget(targetCoord);
        static double currAngles[3];
        control.GetCurrentPose(currAngles);
        
        string angleStr;
        angleStr = "JOINT0: " + to_string(currAngles[0]);
        putText(dst2, angleStr, Point(200, 15 * 1), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        angleStr = "JOINT1: " + to_string(currAngles[1]);
        putText(dst2, angleStr, Point(200, 15 * 2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        angleStr = "JOINT2: " + to_string(currAngles[2]);
        putText(dst2, angleStr, Point(200, 15 * 3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        
        double angles[3];
        control.GetArmPose(angles);
        angleStr = "SET0: " + to_string(angles[0]);
        putText(dst2, angleStr, Point(5, 15 * 4), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        angleStr = "SET1: " + to_string(angles[1]);
        putText(dst2, angleStr, Point(5, 15 * 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        angleStr = "SET2: " + to_string(angles[2]);
        putText(dst2, angleStr, Point(5, 15 * 6), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        
        
        
        
        
        
        
        
        
        ///////
        
        
        Point2f point1;
        Point2f point2;
        
        double yOffset = height * 100;
        int j = reprojectVal;
        for(int i = 0; i < 5; i++)
        {
            //for(int j = 2; j < 3; j++)
            //{
            ReprojectPoints(Point3f(-200 + i * 100, yOffset, -200 + j * 100), point1, point2, cameraMatrix1, cameraMatrix2, translation);
            Scalar pixel = Scalar(255 - i * 50, 255 - j * 50, 255);
            //drawPoints(dst1, dst2, point1, point2, pixel);
            //}
        }
        
        
        imshow("Camera2", dst2);
        imshow("Camera1", dst1);
        
        /*
        
        sum3 += float(clock() - beginTime)/CLOCKS_PER_SEC;
        beginTime = clock();
        */
        
        char ch = waitKey(15);
        if(ch == 'k')
        {
            if(coords.size() > 2)
            {
                Point3f base = coords[0];
                
                base.y -= 100;
                
                coords.insert(coords.begin(), base);
                control.SetArmPose(coords);
                control.CalculateLinkLengths();
                
                double currAngles[3];
                control.GetCurrentPose(currAngles);
                
                control.SetTarget(targetCoord);
                
                double angles[3];
                control.GetArmPose(angles);
            }
        }
        if(ch == 'e')
        {
            CalibrateEnvironment(inputCapture1, inputCapture2);
        }
        else if(ch == 'c')
        {
            destroyAllWindows();
            SetBlobColour(inputCapture1, inputCapture2);
            cerr << "Setting Blob HSV Data" << endl;
            FileStorage bfs(blobFileName, FileStorage::READ);
            
            bfs["HSV_Size"] >> hsvSize;
            detector.clear();
            if(hsvSize.data == NULL)
            {
                cerr << "No BlobHSVColour data" << endl;
                
                BlobHueDetector blobDetector;
                blobDetector.SetDefaultHSVRanges();
                detector.push_back(blobDetector);
            }
            else
            {
                cerr << "Found BlobHSVColour data" << endl;
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
        else if(ch == '1')
        {
            reprojectVal = 0;
        }
        else if(ch == '2')
        {
            reprojectVal = 1;
        }
        else if(ch == '3')
        {
            reprojectVal = 2;
        }
        else if(ch == '4')
        {
            reprojectVal = 3;
        }
        else if(ch == '5')
        {
            reprojectVal = 4;
        }
        else if(ch == '-')
        {
            height++;
        }
        else if(ch == '=')
        {
            height--;
        }
        else if((int)ch == 27)
        {
            return 0;
        }


        //cerr << "Draw on image: " << float(clock() - beginTime)/CLOCKS_PER_SEC << endl;
        
        fcount++;
        //cerr << fcount << endl;
        if(fcount == 100)
        {
            //cerr << "REMAP average: " << sum1/100 << endl;
            //cerr << "DETECT average: " << sum2/100 << endl;
            //cerr << "DRAW average: " << sum3/100 << endl;
            fcount = 0;
            sum1 = sum2 = sum3 = 0;
            //break;
        }
    }
}
