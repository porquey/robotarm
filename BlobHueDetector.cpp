#include "BlobHueDetector.h"


BlobHueDetector::BlobHueDetector():
params(),
iLowH(0),
iHighH(179),
iLowS(0),
iHighS(255),
iLowV(0),
iHighV(255)
{
    params.minThreshold = 80;
    params.maxThreshold = 150;
    
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 150;
    params.maxArea = 10000;
    
    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.2;
    
    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.84;
    
    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;
    
    detector = BetterBlobDetector(params);
}
/*
 bool BlobHueDetector::getBlobData(double &area, double &convexity)
 {
 if(!blobDetected)
 {
 return false;
 }
 
 area = blobArea;
 convexity = blobConvexity;
 
 return true;
 }*/

void BlobHueDetector::setHSVRanges(const HSVRanges range)
{
    iLowH = range.lowH;
    iHighH = range.highH;
    iLowS = range.lowS;
    iHighS = range.highS;
    iLowV = range.lowV;
    iHighV = range.highV;
}

void BlobHueDetector::setDefaultHSVRanges()
{
    iLowH = 0;
    iHighH = 179;
    iLowS = 0;
    iHighS = 255;
    iLowV = 0;
    iHighV = 255;
}

bool BlobHueDetector::getBlobCenter(cv::Mat &src, cv::KeyPoint &keypoint)
{
    cv::Mat hsv, thresh;
    int erosionSize = 3;
    int dilationSize = 4;
    
    cv::cvtColor(src, hsv, CV_BGR2HSV);
    
    cv::inRange(hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), thresh);
    thresh = 255 - thresh;
    
    cv::Mat erosionElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                       cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1),
                                                       cv::Point(erosionSize, erosionSize));
    
    // Apply erosion or dilation on the image
    cv::erode(thresh, thresh, erosionElement);
    
    cv::Mat dilationElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                        cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
                                                        cv::Point(dilationSize, dilationSize));
    
    // Apply erosion or dilation on the image
    cv::dilate(thresh, thresh, dilationElement);
    
    // Apply erosion or dilation on the image
    cv::erode(thresh, thresh, erosionElement);
    
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(thresh, keypoints);
    
    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob/usr/local/lib/libopencv_imgproc.2.4.10.dylib
    //cv::drawKeypoints(src, keypoints, dst, cv::Scalar(0,255,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    int i = 0, k = 0;
    int blobSize = 0;
    for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
        //cout << "size of blob is: " << blobIterator->size << endl;
        //cout << "point is at: " << blobIterator->pt << endl;
        //cv:circle(dst, blobIterator->pt, 1, cv::Scalar(0, 0, 255));
        if(blobIterator->size > blobSize)
        {
            blobSize = blobIterator->size;
            k = i;
        }
        i++;
    }
    if(i == 0)
    {
        return false;
    }
    else
    {
        keypoint = keypoints.at(k);
        double blobArea, blobCircularity, blobInertia, blobPerimeter, blobConvexity;
        detector.getBlobData(k, blobArea, blobCircularity,  blobInertia, blobPerimeter, blobConvexity);
        //blobDetected = true;
        //area = blobArea;
        //convexity = blobConvexity;
        return true;
    }
}
