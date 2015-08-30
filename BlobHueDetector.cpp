#include "BlobHueDetector.h"


BlobHueDetector::BlobHueDetector():
params(),
iLowH(0),
iHighH(179),
iLowS(0),
iHighS(255),
iLowV(0),
iHighV(255),
counter1(5),
counter2(5),
last1(cv::Point(0, 0)),
last2(cv::Point(0, 0))
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
void BlobHueDetector::SetHSVRanges(const HSVRanges range)
{
    iLowH = range.lowH;
    iHighH = range.highH;
    iLowS = range.lowS;
    iHighS = range.highS;
    iLowV = range.lowV;
    iHighV = range.highV;
}

void BlobHueDetector::SetDefaultHSVRanges()
{
    iLowH = 0;
    iHighH = 179;
    iLowS = 0;
    iHighS = 255;
    iLowV = 0;
    iHighV = 255;
}

bool BlobHueDetector::GetBlobCentres(cv::Mat &src1, cv::Mat &src2, cv::KeyPoint &keypoint1, cv::KeyPoint &keypoint2)
{
    bool detected1 = false, detected2 = false;
    cv::Rect roi1, roi2;
    if(counter1 < 5)
    {
        int x = last1.x - 50;
        int y = last1.y - 50;
        int width = 100;
        int height = 100;

        if(last1.x - 50 < 0)
        {
            x = 0;
        }
        if(last1.y - 50 < 0)
        {
            y = 0;
        }
        if(last1.x + 50 > src1.cols)
        {
            width = src1.cols - last1.x + 50;
        }
        if(last1.y + 50 > src1.rows)
        {
            height = src1.rows - last1.y + 50;
        }
        roi1 = cv::Rect(x, y, width, height);
        cv::Mat roiMat = src1(roi1);
        detected1 = GetBlob(roiMat, keypoint1);
        keypoint1.pt.x += roi1.x;
        keypoint1.pt.y += roi1.y;
        if(detected1)
        {
            counter1 = 0;
            last1 = keypoint1.pt;
        }
        else
        {
            counter1++;
        }
    }
    else
    {
        detected1 = GetBlob(src1, keypoint1);
        if(detected1)
        {
            counter1 = 0;
            last1 = keypoint1.pt;
        }
    }
    
    if(counter2 < 5)
    {
        int x = last2.x - 50;
        int y = last2.y - 50;
        int width = 100;
        int height = 100;
        
        if(last2.x - 50 < 0)
        {
            x = 0;
        }
        if(last2.y - 50 < 0)
        {
            y = 0;
        }
        if(last2.x + 50 > src2.cols)
        {
            width = src2.cols - last2.x + 50;
        }
        if(last2.y + 50 > src2.rows)
        {
            height = src2.rows - last2.y + 50;
        }
        roi2 = cv::Rect(x, y, width, height);
        cv::Mat roiMat = src2(roi2);
        detected2 = GetBlob(roiMat, keypoint2);
        keypoint2.pt.x += roi2.x;
        keypoint2.pt.y += roi2.y;
        if(detected2)
        {
            counter2 = 0;
            last2 = keypoint2.pt;
        }
        else
        {
            counter2++;
        }
    }
    else
    {
        detected2 = GetBlob(src2, keypoint2);
        if(detected2)
        {
            counter2 = 0;
            last2 = keypoint2.pt;
        }
    }
    return detected1 && detected2;
}

bool BlobHueDetector::GetBlob(cv::Mat &src, cv::KeyPoint &keypoint)
{
    cv::Mat hsv, thresh;
    cv::cvtColor(src, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), thresh);
    thresh = 255 - thresh;
    int erosionSize = 3;
    int dilationSize = 4;
    
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
    
    int i = 0, k = 0;
    int blobSize = 0;
    if(keypoints.size() == 0)
    {
        return false;
    }
    else
    {
        for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
            
            if(blobIterator->size > blobSize)
            {
                blobSize = blobIterator->size;
                k = i;
            }
            i++;
        }
        keypoint = keypoints.at(k);
        return true;
    }

    
}




