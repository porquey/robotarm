#include "BlobHueDetector.h"


BlobHueDetector::BlobHueDetector():
params(),
iLowH(56),
iHighH(76),
iLowS(0),
iHighS(255),
iLowV(40),
iHighV(255),
blobDetected(false),
blobArea(0),
blobConvexity(0)
{
    params.minThreshold = 80;
    params.maxThreshold = 150;
    
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 200;
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

void BlobHueDetector::setHSVRanges(const int lowH, const int highH, const int lowS, const int highS, const int lowV, const int highV)
{
    iLowH = lowH;
    iHighH = highH;
    iLowS = lowS;
    iHighS = highS;
    iLowV = lowV;
    iHighV = highV;
}

bool BlobHueDetector::getBlobCenter(cv::Mat &src, cv::Mat &thresh, cv::Point2f &center, cv::Mat &dst, std::string windowName, double &area, double &convexity)
{
    cv::Mat hsv;
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
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    cv::drawKeypoints(src, keypoints, dst, cv::Scalar(0,255,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    int i = 0, k = 0;
    int blobSize = 0;
    for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
        //cout << "size of blob is: " << blobIterator->size << endl;
        //cout << "point is at: " << blobIterator->pt << endl;
        cv:circle(dst, blobIterator->pt, 1, cv::Scalar(0, 0, 255));
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
        center = keypoints.at(k).pt;
        detector.getBlobData(k, blobArea, blobCircularity,  blobInertia, blobPerimeter, blobConvexity);
        blobDetected = true;
        area = blobArea;
        convexity = blobConvexity;
        return true;
    }
}
