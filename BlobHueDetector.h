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
    bool GetBlobCentres(cv::Mat &src1, cv::Mat &src2, cv::KeyPoint &keypoint1, cv::KeyPoint &keypoint2);
    bool GetBlob(cv::Mat &src, cv::KeyPoint &keypoint);
    //bool getBlobData(double &area, double &convexity);
    void SetHSVRanges(const HSVRanges range);
    void SetDefaultHSVRanges();
private:
    BetterBlobDetector detector;
    cv::SimpleBlobDetector::Params params;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    int counter1, counter2;
    cv::Point last1, last2;
    
    //bool blobDetected;
    //double blobArea;
    //double blobCircularity;
    //double blobInertia;
    //double blobPerimeter;
    //double blobConvexity;
};
