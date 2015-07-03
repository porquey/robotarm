#include "opencv2/opencv.hpp"
#include "BetterBlobDetector.h"
#include <iostream>

class BlobHueDetector
{
public:
    
    BlobHueDetector();
    /*
     BlobHueDetector(cv::SimpleBlobDetector::Params &parameters);
     
     BlobHueDetector(cv::SimpleBlobDetector::Params &parameters, int lowH, int highH, int lowS, int highS, int lowV, int highV);
     */
    bool getBlobCenter(cv::Mat &src, cv::Mat &thresh, cv::Point2f &center, cv::Mat &dst, std::string windowName, double &area, double &convexity);
    //bool getBlobData(double &area, double &convexity);
    void setHSVRanges(const int lowH, const int highH, const int lowS, const int highS, const int lowV, const int highV);
private:
    BetterBlobDetector detector;
    cv::SimpleBlobDetector::Params params;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    
    bool blobDetected;
    double blobArea;
    double blobCircularity;
    double blobInertia;
    double blobPerimeter;
    double blobConvexity;
};
