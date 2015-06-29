#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include "BlobHueDetector.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define HALF_POINT_X 319.5
#define HALF_POINT_Y 239.5


using namespace cv;
using namespace std;

static int iLowH = 0;
static int iHighH = 179;

static int iLowS = 0;
static int iHighS = 255;

static int iLowV = 0;
static int iHighV = 255;

void MouseCallBack(int event, int x, int y, int flags, void* userData)
{
    if(event == EVENT_LBUTTONDOWN)
    {
        Mat* img = static_cast<Mat*> (userData);
        cout << "X: " << x << " Y:" << y << endl;
        Vec3b pixel = img->at<Vec3b>(y, x);
        cout << "H: " << (int)pixel[0] << " S: " << (int)pixel[1] << " V: " << (int)pixel[2] << endl;
        iLowH = (int)pixel[0] - 10;
        iHighH = (int)pixel[0] + 10;
        iLowS = (int)pixel[1] - 80;
        iHighS = (int)pixel[1] + 80;
        iLowV = (int)pixel[2] - 70;
        iHighV = (int)pixel[2] + 70;
    }
}

int main( int argc, char** argv )
{
  float xPos = 0, yPos = 0, zPos = 0, xPosLast, yPosLast, zPosLast;
  float fx1, fy1, cx1, cy1, fx2, fy2, cx2, cy2, xTrans, yTrans, zTrans;
    xPosLast = 0;
    yPosLast = 0;
    zPosLast = 0;

  const string fileName = "EnvironmentCalibration.xml";
  BlobHueDetector detector;

  cout << "Attempting to open configuration files" << endl;
  FileStorage fs(fileName, FileStorage::READ);

  Mat cameraMatrix1, cameraMatrix2, mapX1, mapY1, mapX2, mapY2, translation;

  fs["Camera_Matrix_1"] >> cameraMatrix1;
  fs["Camera_Matrix_2"] >> cameraMatrix2;
  fs["Mapping_X_1"] >> mapX1;
  fs["Mapping_Y_1"] >> mapY1;
  fs["Mapping_X_2"] >> mapX2;
  fs["Mapping_Y_2"] >> mapY2;
  fs["Translation"] >> translation;

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

/*  cout << "fx1: "<< fx1 << endl;
  cout << "cx1: "<< cx1 << endl;
  cout << "fy1: "<< fy1 << endl;
  cout << "cy1: "<< cy1 << endl;
  cout << "fx2: "<< fx2 << endl;
  cout << "cx2: "<< cx2 << endl;
  cout << "fy2: "<< fy2 << endl;
  cout << "fc2: "<< cy2 << endl;

  cout << "translationX: " << xTrans << endl;
  cout << "translationY: " << yTrans << endl;
  cout << "translationZ: " << zTrans << endl;*/

  VideoCapture inputCapture1(1);
  inputCapture1.set(CV_CAP_PROP_FRAME_WIDTH,640);
  inputCapture1.set(CV_CAP_PROP_FRAME_HEIGHT,480);

  VideoCapture inputCapture2(2);
  inputCapture2.set(CV_CAP_PROP_FRAME_WIDTH,640);
  inputCapture2.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    namedWindow("Src", WINDOW_AUTOSIZE);
  Mat image1, image2;
    bool colourSelected = false;
  while(inputCapture1.isOpened() && inputCapture2.isOpened() && !colourSelected){
    inputCapture1.read(image1);
    inputCapture2.read(image2);
      imshow("Src", image1);
      Mat hsv, thresh;
      cvtColor(image1, hsv, CV_BGR2HSV);
      void* userData = static_cast<void*>(&hsv);
      setMouseCallback("Src", MouseCallBack, userData);

      inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh);
      thresh = 255 - thresh;
      Mat erosionElement = getStructuringElement(cv::MORPH_ELLIPSE,
                                                 cv::Size(2 * 3 + 1, 2 * 3 + 1),
                                                 cv::Point(3, 3) );
      
      // Apply erosion or dilation on the image
      erode(thresh, thresh, erosionElement);
      
      Mat dilationElement = getStructuringElement(cv::MORPH_ELLIPSE,
                                                  cv::Size(2 * 4 + 1, 2 * 4 + 1),
                                                  cv::Point(4, 4) );
      
      
      // Apply erosion or dilation on the image
      dilate(thresh, thresh, dilationElement);
      
      
      imshow("Thresh", thresh);
      
      int key = waitKey(30);
      
     if((char)key == 'q')
      {
          iHighH += 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
          
      }
      else if((char)key == 'w')
      {
          iLowH += 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == 'e')
      {
          iHighS += 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == 'r')
      {
          iLowS += 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == 't')
      {
          iHighV += 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == 'y')
      {
          iLowV += 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }else if((char)key == 'a')
      {
          iHighH -= 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
          
      }
      else if((char)key == 's')
      {
          iLowH -= 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == 'd')
      {
          iHighS -= 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == 'f')
      {
          iLowS -= 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == 'g')
      {
          iHighV -= 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == 'h')
      {
          iLowV -= 5;
          cout << "H: " << iHighH << " - " << iLowH << endl;
          cout << "S: " << iHighS << " - " << iLowS << endl;
          cout << "V: " << iHighV << " - " << iLowV << endl;
      }
      else if((char)key == ' ')
      {
          colourSelected = true;
          detector.setHSVRanges(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
      }
      
  }
    destroyAllWindows();
    string posStr = "X: " + to_string(xPos) + "\tY: " + to_string(yPos) + "\tZ: " + to_string(zPos);
    while(inputCapture1.isOpened() && inputCapture2.isOpened()){
    inputCapture1.read(image1);
    inputCapture2.read(image2);
    Mat t1 = image1.clone();
    Mat t2 = image2.clone();
    remap(t1, image1, mapX1, mapY1, INTER_LINEAR);
    remap(t2, image2, mapX2, mapY2, INTER_LINEAR);
    t1.release();
    t2.release();
    Point2f center1, center2;
    Mat thresh1, thresh2, dst1, dst2;
        double area1, area2, conv1, conv2;
    bool detected1 = detector.getBlobCenter(image1, thresh1, center1, dst1, "Cam1", area1, conv1);
    bool detected2 = detector.getBlobCenter(image2, thresh2, center2, dst2, "Cam2", area2, conv2);
        imshow("Thresh1", thresh1);
        imshow("Thresh2", thresh2);
        float x1 = center1.x - cx1;
        float x2 = center2.x - cx2;
        float y1 = center1.y - cy1;
        float y2 = center2.y - cy2;
        
        
        float a = (x1)/fx1;
        float b = (x2)/fx2;
        float c = a*b;
        
        xPos = (c*xTrans + a*zTrans)/(1+c);
        zPos = (b*xTrans + zTrans)/(1+c);
        yPos = -((y1)/fy1) * zPos;
        zPos -= zTrans;
        
        posStr = "X: " + to_string(xPos) + "  Y: " + to_string(yPos) + "  Z: " + to_string(zPos);
        

    
        putText(dst1, posStr, Point(5,15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        
        
        imshow("Camera2", dst2);
        imshow("Camera1", dst1);

    char ch = waitKey(15);
    if (ch == 'p'){
        

        
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
