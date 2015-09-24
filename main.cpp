#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SetBlobColour.h"
#include "BlobHueDetector.h"
#include "CalibrateEnvironment.h"
#include "ControlArm.h"
#include "JointPositions.h"
#include "GraphRecorder.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define CAMERA1 1
#define CAMERA2 2

#define LINK0 170
#define LINK1 200
#define LINK2 185


using namespace cv;
using namespace std;

float fx1, fy1, cx1, cy1, fx2, fy2, cx2, cy2, xTrans, yTrans, zTrans;

double sum1 = 0, sum2 = 0, sum3 = 0;
int fcount = 0;

int reprojectVal = 0, yOff = 0, xOff = 0, zOff = 0;

bool checkBlobs = true;
bool targetExists = false;
bool targetToggle = false;
Point3f targetCoord;

Mat cameraMatrix1, cameraMatrix2, mapX1, mapY1, mapX2, mapY2, translation;

double l0 = LINK0, l1 = LINK1, l2 = LINK2;


void Draw3DPoint(Point3f pt, Mat &dst1, Mat &dst2)
{
    Point2f a, b;
    ReprojectPoints(pt, a, b, cameraMatrix1, cameraMatrix2, translation);
    
    circle(dst1, a, 8, Scalar(255, 255, 0), 2);
    circle(dst2, b, 8, Scalar(255, 255, 0), 2);
}

void Draw3DLine(Point3f pt1, Point3f pt2, Mat &dst1, Mat &dst2)
{
    Point2f a1, b1, a2, b2;
    ReprojectPoints(pt1, a1, b1, cameraMatrix1, cameraMatrix2, translation);
    ReprojectPoints(pt2, a2, b2, cameraMatrix1, cameraMatrix2, translation);
    
    line(dst1, a1, a2, Scalar(255, 255, 0), 2);
    line(dst2, b1, b2, Scalar(255, 255, 0), 2);
}

void DrawInverseKinematics(vector<Point3f> coords, double* angles, Point3f temp, Mat &dst1, Mat &dst2)
{
    Point3f basej = coords[1];
    double tempAngle0, tempAngle1, tempAngle2;
    if(temp.z > coords[0].z)
    {
        tempAngle0 = angles[0];
        tempAngle1 = -angles[1];
        tempAngle2 = -angles[2];
    }
    else
    {
        tempAngle0 = angles[0];
        tempAngle1 = angles[1];
        tempAngle2 = angles[2];
    }
    
    double height = l1 * sin(HALF_PI - tempAngle1);
    double width = l1 * cos(HALF_PI - tempAngle1);
    
    Point3f jj1 = Point3f(basej.x - cos(tempAngle0) * width, height + basej.y, basej.z - sin(tempAngle0) * width);
    Draw3DPoint(basej, dst1, dst2);
    Draw3DPoint(jj1, dst1, dst2);
    Draw3DLine(basej, jj1, dst1, dst2);
    Draw3DLine(basej, coords[0], dst1, dst2);
    
    Point3f baseVec = temp - coords[0];
    baseVec.y = 0;
    
    int d = FindAngleDirection(baseVec, basej - jj1, temp - jj1);
    
    double extraHeight = l2 * sin(HALF_PI - tempAngle1 + tempAngle2 * d);
    double extraWidth = l2 * cos(HALF_PI - tempAngle1 + tempAngle2 * d);
    Point3f jj2 = Point3f(jj1.x - cos(tempAngle0) * extraWidth, jj1.y + extraHeight, jj1.z - sin(tempAngle0) * extraWidth);
    Draw3DPoint(jj2, dst1, dst2);
    Draw3DLine(jj1, jj2, dst1, dst2);
}

int main(int argc, char** argv)
{
    
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
    
    Point3f tempTarget;
    
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
        if(targetData.data != NULL)
        {
            HSVRanges targetRanges;
            targetRanges.lowH = targetData.at<int>(0, 0);
            targetRanges.highH = targetData.at<int>(0, 1);
            targetRanges.lowS = targetData.at<int>(1, 0);
            targetRanges.highS = targetData.at<int>(1, 1);
            targetRanges.lowV = targetData.at<int>(2, 0);
            targetRanges.highV = targetData.at<int>(2, 1);
            
            targetDetector.SetHSVRanges(targetRanges);
            targetExists = true;
        }
        else
        {
            targetDetector.SetDefaultHSVRanges();
        }
    }
    

    
    cerr << "Initialising" << endl;
    Mat thresh1, thresh2, dst1, dst2;

    clock_t beginTime = clock();
    clock_t intervalTime = 100000;
    clock_t startTime = 0;
    //int frames = 0;
    ControlArm control(LINK0, LINK1, LINK2);
    ControlArm::PIDControl pid0 = ControlArm::PIDControl();
    ControlArm::PIDControl pid1 = ControlArm::PIDControl();
    ControlArm::PIDControl pid2 = ControlArm::PIDControl();
    double destAngle = 0;
    bool rampEnabled = false;
    bool pidEnabled = false;
    bool delayPassed = false;
    
    GraphRecorder recorder1,recorder2,recorder3,recorder4,recorder5;
    RampValue ramp1, ramp2, ramp3;
    
    string fpsStr;
    
    while(inputCapture1.isOpened() && inputCapture2.isOpened())
    {
        beginTime = clock();
        //frames++;
        
        
        // Read and transform images from cameras
        inputCapture1.read(image1);
        inputCapture2.read(image2);
        Mat t1 = image1.clone();
        Mat t2 = image2.clone();
        remap(t1, image1, mapX1, mapY1, INTER_LINEAR);
        remap(t2, image2, mapX2, mapY2, INTER_LINEAR);
        t1.release();
        t2.release();
        
        vector<bool> detectedVec;
        vector<Point3f> coords, coordsLast;
        static Point3f base, tip;
        
        static Point3f joint1, joint2;
        static Point3f begin0, begin1, begin2, end0, end1, end2;

        dst1 = image1.clone();
        dst2 = image2.clone();
        
        if(checkBlobs)
        {
            vector<KeyPoint> keypointVec1, keypointVec2;
            
            for(int i = 0; i < blobNum; i++)
            {
                KeyPoint keypoint1, keypoint2;
                vector<Mat> imageVec;
                imageVec.push_back(image1);
                imageVec.push_back(image2);
                vector<KeyPoint> keypointVec;

                bool detected = detector[i].GetJointPos(imageVec, keypointVec);
                
               
                
                
                Point3f coordTemp = Calculate3DPoint(keypointVec[0].pt, keypointVec[1].pt, cameraMatrix1, cameraMatrix2, translation);
                
                coords.push_back(coordTemp);
                detectedVec.push_back(detected);
                keypointVec1.push_back(keypointVec[0]);
                keypointVec2.push_back(keypointVec[1]);
                string posStr = "X: " + to_string(coordTemp.x) + "  Y: " + to_string(coordTemp.y) + "  Z: " + to_string(coordTemp.z);
                //putText(dst1, posStr, Point(5, 15 * (i + 1)), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
            }
            
            cv::drawKeypoints(dst1, keypointVec1, dst1, cv::Scalar(0, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(dst2, keypointVec2, dst2, cv::Scalar(0, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        }
        
        KeyPoint keypoint1, keypoint2;
        if(targetExists)
        {
            vector<KeyPoint> targetVec1, targetVec2;
            targetDetector.GetBlobCentres(image1, image2, keypoint1, keypoint2);
            targetCoord = Calculate3DPoint(keypoint1.pt, keypoint2.pt, cameraMatrix1, cameraMatrix2, translation);
            targetVec1.push_back(keypoint1);
            targetVec2.push_back(keypoint2);

            cv::drawKeypoints(dst1, targetVec1, dst1, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(dst2, targetVec2, dst2, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            
            string posStr = "X: " + to_string(targetCoord.x) + "  Y: " + to_string(targetCoord.y) + "  Z: " + to_string(targetCoord.z);

        }
        
        /////CONTROL/////
        Point2f pt1, pt2;
        ReprojectPoints(tempTarget, pt1, pt2, cameraMatrix1, cameraMatrix2, translation);
        circle(dst1, pt1, 10, Scalar(255, 255, 255), 2);
        circle(dst2, pt2, 10, Scalar(255, 255, 255), 2);
        string posStr = "TARGET X: " + to_string(tempTarget.x) + "  Y: " + to_string(tempTarget.y) + "  Z: " + to_string(tempTarget.z);
        putText(dst1, posStr, Point(5, 15 * 1), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        posStr = "END EFFECTOR X: " + to_string(coords[3].x) + "  Y: " + to_string(coords[3].y) + "  Z: " + to_string(coords[3].z);
        putText(dst1, posStr, Point(5, 15 * 2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        //cerr << base << " " << joint1 << " " << joint2 << " " << tip << endl;
        control.SetArmPose(coords);
        if(targetExists && targetToggle)
        {
            control.SetTarget(targetCoord);
        }
        else
        {
            control.SetTarget(tempTarget);
        }
        static double currAngles[3];
        control.GetCurrentPose(currAngles);
        
        
        control.UpdateArmPose(pid0.GetLastError(), pid1.GetLastError(), pid2.GetLastError());

        
        string angleStr;

        
        angleStr = "JOINT0: " + to_string(currAngles[0]);
        putText(dst2, angleStr, Point(5, 15 * 1), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        angleStr = "JOINT1: " + to_string(currAngles[1]);
        putText(dst2, angleStr, Point(5, 15 * 2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        angleStr = "JOINT2: " + to_string(currAngles[2]);
        putText(dst2, angleStr, Point(5, 15 * 3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        
        double angles[3];
        control.GetArmPose(angles);
        angleStr = "SET0: " + to_string(angles[0]);
        putText(dst2, angleStr, Point(5, 15 * 4), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        angleStr = "SET1: " + to_string(angles[1]);
        putText(dst2, angleStr, Point(5, 15 * 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        angleStr = "SET2: " + to_string(angles[2]);
        putText(dst2, angleStr, Point(5, 15 * 6), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
         
        bool inRange = true;
        for (int i = 0; i < 3; i++){
            if (currAngles[i] < -3.14 || currAngles[i] > 3.14 )
            {
                inRange = false;
                cerr << "JOINT " << i << "OUT OF RANGE: " << currAngles[i] << endl;
            }
        }
        static bool hacky = false;
        static bool starty = false;
        if (inRange)
        {
            if(hacky)
            {
                if(!starty)
                {
                    if(targetExists && targetToggle)
                    {
                        control.SetTarget(targetCoord);
                    }
                    else
                    {
                        yOff = -2;
                        xOff = -2;
                        zOff = -2;
                        double yOffset = yOff * 100;
                        double xOffset = xOff * 100;
                        double zOffset = zOff * 100;
                        tempTarget = Point3f(xOffset, yOffset, zOffset);
                        control.SetTarget(tempTarget);
                    }
                }
                else
                {
                    if(targetExists && targetToggle)
                    {
                        control.SetTarget(targetCoord);
                    }
                    else
                    {
                        double yOffset = yOff * 100;
                        double xOffset = xOff * 100;
                        double zOffset = zOff * 100;
                        tempTarget = Point3f(xOffset + ramp1.getCurrentValue(), yOffset + ramp2.getCurrentValue(), zOffset);// + ramp3.getCurrentValue());
                        control.SetTarget(tempTarget);
                    }
                    
                }
                control.SendJointActuators(pid0.Update(currAngles[0], angles[0]),pid1.Update(currAngles[1], angles[1]), pid2.Update(currAngles[2], angles[2]));
            }
            else
            {
                control.SendJointActuators(0,pid1.Update(currAngles[1], 0), pid2.Update(currAngles[2], 0));
            }
        }
        else{
            cerr << "COULD NOT DETECT" << endl;
            control.SendJointActuators(0, 0, 0);
        }
        
        recorder1.writeValue(tempTarget.x, coords[3].x, (double)clock());
        recorder2.writeValue(tempTarget.y, coords[3].y, (double)clock());
        recorder3.writeValue(tempTarget.z, coords[3].z, (double)clock());
        recorder4.writeValue(CalculateLength(tempTarget - coords[3]), (double)clock());
        recorder5.writeValue(CalculateLength(control.GetFuzzyTarget() - coords[3]), (double)clock());
        
        
        DrawInverseKinematics(coords, angles, control.GetFuzzyTarget(), dst1, dst2);
        
        
        putText(dst1, fpsStr, Point(5, 15 * 3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

        imshow("Camera2", dst2);
        imshow("Camera1", dst1);
        
        while((clock()-beginTime) < intervalTime){};
        fpsStr = "FPS: " + to_string(clock()-beginTime);
//        cerr << fpsStr << endl;
        
        if ((clock() - startTime) > 5000000 && startTime != 0){
            delayPassed = true;
        }

        
        char ch = waitKey(15);
        if(ch == 'l')
        {
            control.CalculateLinkLengths(l0, l1, l2);
        }
        else if(ch == 'r')
        {
            control.GetError();
        }
        else if(ch == 't')
        {
            targetToggle = !targetToggle;
        }
        else if(ch == 'h')
        {
            control.InitFuzzyController();
            hacky = true;
        }
        else if(ch == 'f')
        {
            control.TerminateFuzzyController();
        }
        else if(ch == 'e')
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
                if(targetData.data != NULL)
                {
                    HSVRanges targetRanges;
                    targetRanges.lowH = targetData.at<int>(0, 0);
                    targetRanges.highH = targetData.at<int>(0, 1);
                    targetRanges.lowS = targetData.at<int>(1, 0);
                    targetRanges.highS = targetData.at<int>(1, 1);
                    targetRanges.lowV = targetData.at<int>(2, 0);
                    targetRanges.highV = targetData.at<int>(2, 1);
                    
                    targetDetector.SetHSVRanges(targetRanges);
                }
                else
                {
                    targetDetector.SetDefaultHSVRanges();
                }
            }
            destroyAllWindows();
        }
        else if(ch == '1')
        {
            yOff = -2;
            xOff = 2;
            zOff = -2;
            double yOffset = yOff * 100;
            double xOffset = xOff * 100;
            double zOffset = zOff * 100;
            tempTarget = Point3f(xOffset, yOffset, zOffset);
        }
        else if((int)ch == 27)
        {
            return 0;
        }
        else if(ch == 'a')
        {
            destAngle = 0.3;
            cerr << "PID enabled. Target : " << destAngle << endl;
            pid1.Reset();
            pid0.Reset();
            pid2.Reset();
            pidEnabled = true;
            delayPassed = false;
        }
        else if(ch == 'b')
        {
            cerr << "Ramp and PID disabled" << endl;
            pidEnabled = false;
            rampEnabled = false;
            control.SendJointActuators(0,0,0);
//            cerr << "PID enabled. Target : " << destAngle << endl;
//            destAngle = -1.5;
//            PIDEnabled = true;
//            pid1.reset();
//            pid0.reset();
//            pid2.reset();
        }
        else if (ch == 's')
        {
            //ramp.start(HALF_PI, 15000000, 5000000);
            starty = true;
            //control.InitFuzzyController();

            cerr << "Step started" << destAngle << endl;
            startTime = clock();
            recorder1.start("x.txt", 40000000);
            recorder2.start("y.txt", 40000000);
            recorder3.start("z.txt", 40000000);
            recorder4.start("err.txt", 45000000);
            recorder5.start("fuzz_err.txt", 45000000);
            
            ramp1.start(400, 8000000, 5000000);
            ramp2.start(100, 8000000, 5000000);
            ramp3.start(400, 8000000, 5000000);

        }
        
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
