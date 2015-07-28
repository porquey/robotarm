#include "opencv2/opencv.hpp"
#include "BetterBlobDetector.h"
#include "HSVRange.h"
#include <iostream>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

class BlobHueDetector
{
public:
    
    BlobHueDetector();
    /*
     BlobHueDetector(cv::SimpleBlobDetector::Params &parameters);
     
     BlobHueDetector(cv::SimpleBlobDetector::Params &parameters, int lowH, int highH, int lowS, int highS, int lowV, int highV);
     */
    bool getBlobCenter(cv::Mat &src, cv::KeyPoint &keypoint);
    //bool getBlobData(double &area, double &convexity);
    void setHSVRanges(const HSVRanges range);
    void setDefaultHSVRanges();
private:
    BetterBlobDetector detector;
    cv::SimpleBlobDetector::Params params;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    
    //bool blobDetected;
    //double blobArea;
    //double blobCircularity;
    //double blobInertia;
    //double blobPerimeter;
    //double blobConvexity;
};
