#include "BlobHueDetector.h"

/// BlobHueDetector
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
last1(),
last2(),
lastBegin1(),
lastEnd1(),
lastBegin2(),
lastEnd2(),
countVec(),
offsetVec()
{
    // threshold range
    params.minThreshold = 80;
    params.maxThreshold = 110;
    
    // filter by area.
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 15000;
    
    // filter options
    params.filterByCircularity = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;
    params.filterByColor = false;
    
    // filter by distance
    params.minDistBetweenBlobs = 40;
    
    // initialisation
    detector = BetterBlobDetector(params);
    countVec.push_back(5);
    countVec.push_back(5);
    offsetVec.push_back(KeyPoint());
    offsetVec.push_back(KeyPoint());
    filter[0].Init();
    filter[1].Init();
}

/// SetHSVRanges
/// In: range: HSV range values
void BlobHueDetector::SetHSVRanges(const HSVRanges range)
{
    iLowH = range.lowH;
    iHighH = range.highH;
    iLowS = range.lowS;
    iHighS = range.highS;
    iLowV = range.lowV;
    iHighV = range.highV;
}

/// SetDefaultHSVRanges
void BlobHueDetector::SetDefaultHSVRanges()
{
    iLowH = 0;
    iHighH = 179;
    iLowS = 0;
    iHighS = 255;
    iLowV = 0;
    iHighV = 255;
}

/// GetBlobCentres
/// In: src1: image from camera1
///     src2: image from camera2
/// Out: keypoint1: point and size of detected blob in camera1
///      keypoint2: point and size of detected blob in camera2
bool BlobHueDetector::GetBlobCentres(Mat &src1, Mat &src2, KeyPoint &keypoint1, KeyPoint &keypoint2)
{
    // booleans to check if each camera has detected the blob
    bool detected1 = false, detected2 = false;
    
    // call GetBlob to detect blob in camera1
    detected1 = GetBlob(src1, keypoint1);
    if(detected1)
    {
        counter1 = 0;
        last1 = keypoint1;
    }
    
    // call GetBlob to detect blob in camera2
    detected2 = GetBlob(src2, keypoint2);
    if(detected2)
    {
        counter2 = 0;
        last2 = keypoint2;
    }
    
    // if not detected use last detected blob
    if(!detected1 || !detected2)
    {
        keypoint1 = last1;
        keypoint2 = last2;
    }
    return detected1 && detected2;
}

/// GetBlob
/// In: src: source image
/// Out: keypoint: point and size of detected blob
bool BlobHueDetector::GetBlob(Mat &src, KeyPoint &keypoint)
{
    // convert rgb to hsv
    Mat hsv, thresh;
    cvtColor(src, hsv, CV_BGR2HSV);
    
    // wrap around hue value from 0 to 180
    if(iLowH > iHighH)
    {
        Mat temp1, temp2;
        inRange(hsv, Scalar(0, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), temp1);
        inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(179, iHighS, iHighV), temp2);
        bitwise_or(temp1, temp2, thresh);
    }
    else
    {
        inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh);
    }

    // invert image
    thresh = 255 - thresh;
    
    // set erosion and dilation kernel size
    int erosionSize = 3;
    int dilationSize = 4;
    
    // apply erosion and dilation
    ApplyMorphologicalOperation(thresh, erosionSize, dilationSize);

    // detect keypoints
    vector<KeyPoint> keypoints;
    detector.detect(thresh, keypoints);
    
    // find largest blob
    int i = 0, k = 0;
    double blobSize = 0;
    if(keypoints.size() == 0)
    {
        return false;
    }
    else
    {
        for(vector<KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
            
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

/// GetJointPos
/// In: srcVec: images from the set of cameras
/// Out: pointVec: vector of detected points from the images
bool BlobHueDetector::GetJointPos(vector<Mat> &srcVec, vector<KeyPoint> &pointVec)
{
    Mat thresh;
    Rect roi;
    const int margin = 75;
    vector<bool> detectedVec;
    
    // iterate over each camera
    for(int i = 0; i < srcVec.size(); i++)
    {
        // check if 5 frames has been processed since last detection, if not only search in a
        // small region of interest
        if(countVec[i] < 5)
        {
            // extract region of interest
            int xLeft = offsetVec[i].pt.x - margin;
            int xRight = offsetVec[i].pt.x + margin;
            int yTop = offsetVec[i].pt.y - margin;
            int yBottom = offsetVec[i].pt.y + margin;
            if(xLeft < 0)
            {
                xLeft = 0;
            }
            if(xRight > srcVec[i].cols - 1)
            {
                xRight = srcVec[i].cols - 1;
            }
            if(yTop < 0)
            {
                yTop = 0;
            }
            if(yBottom > srcVec[i].rows - 1)
            {
                yBottom = srcVec[i].rows - 1;
            }
            roi = Rect(xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1);
            Mat roiMat = srcVec[i](roi);
            
            // search for joint over region of interest
            KeyPoint point, avg;
            detectedVec.push_back(GetJointBlob(roiMat, point, thresh));
            point.pt.x += roi.x;
            point.pt.y += roi.y;
            
            // add detected point to moving average filter
            if(detectedVec[i])
            {
                countVec[i] = 0;
                offsetVec[i] = point;
                avg.pt = filter[i].Add(point.pt);
                avg.size = point.size;
            }
            else
            {
                countVec[i]++;
                avg.pt = filter[i].Average();
                avg.size = offsetVec[i].size;
            }
            pointVec.push_back(avg);

        }
        else
        {
            // if not detected for more than 5 frames, perform the detection over while image
            KeyPoint point, avg;
            detectedVec.push_back(GetJointBlob(srcVec[i], point, thresh));
            
            // add to moving average filter
            if(detectedVec[i])
            {
                countVec[i] = 0;
                offsetVec[i] = point;
                avg.pt = filter[i].Add(point.pt);
                avg.size = point.size;
            }
            else
            {
                avg.pt = filter[i].Average();
                avg.size = offsetVec[i].size;
            }
            pointVec.push_back(avg);
        }
    }
    
    if(detectedVec.size() < 2)
    {
        return false;
    }
    else
    {
        return detectedVec[0] && detectedVec[1];
    }
}

/// GetJointBlob
/// In: src: source image
/// Out: point: detected point and size
///      thresh: image after binary threshold for debugging purposes
bool BlobHueDetector::GetJointBlob(Mat &src, KeyPoint &point, Mat &thresh)
{
    // convert image to HSV colourspace
    Mat hsv;
    cvtColor(src, hsv, CV_BGR2HSV);
    
    // wrap around for hue value from 0 to 180
    if(iLowH > iHighH)
    {
        Mat temp1, temp2;
        inRange(hsv, Scalar(0, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), temp1);
        inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(179, iHighS, iHighV), temp2);
        bitwise_or(temp1, temp2, thresh);
    }
    else
    {
        inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh);
    }    thresh = 255 - thresh;
    
    // set erosion and dilation kernel size
    int erosionSize = 3;
    int dilationSize = 3;
    
    //apply erosion and dilation
    ApplyMorphologicalOperation(thresh, erosionSize, dilationSize);
    
    // search for joints in images
    vector<KeyPoint> keypoints;
    detector.detect(thresh, keypoints);
    
    // check if anything was detected
    int k = 0;
    double blobSize = 0;
    if(keypoints.size() == 0)
    {
        return false;
    }
    else
    {
        // search for largest detection
        for(int i = 0; i < keypoints.size(); i++)
        {
            if(keypoints[i].size > blobSize)
            {
                blobSize = keypoints[i].size;
                k = i;
            }
        }
        
        // add nearby blobs to cluster
        vector<KeyPoint> cluster;
        cluster.push_back(keypoints.at(k));
        for(int i = 0; i < keypoints.size(); i++)
        {
            if(i != k)
            {
                double distance = sqrt((keypoints[i].pt.x - keypoints[k].pt.x) * (keypoints[i].pt.x - keypoints[k].pt.x) + (keypoints[i].pt.y - keypoints[k].pt.y) * (keypoints[i].pt.y - keypoints[k].pt.y));
                if(distance < 60)
                {
                    cluster.push_back(keypoints[i]);
                }
            }
        
        }
        
        // calculate the weighted average point between the blobs
        Point2f avg = Point2f(0, 0);
        double sum = 0;
        for(int i = 0; i < cluster.size(); i++)
        {
            avg = avg + cluster[i].pt * cluster[i].size;
            sum += cluster[i].size;
        }
        double denom = 1/sum;
        avg = avg * denom;
        point = KeyPoint(avg, sum);
        return true;
    }
}

/// GetStripVectors
/// This function was originally used when each link was equipped with strips of colour
/// to calculate the vector and hence calculate the joint positions.
/// However it was replaced by the GetJointPos function for its reliability and robustness.
/*
 bool BlobHueDetector::GetStripVectors(Mat &src1, Mat &src2, KeyPoint &begin1, KeyPoint &end1, KeyPoint &begin2, KeyPoint &end2)
 {
 bool detected1 = false, detected2 = false;
 Mat thresh1, thresh2;
 Rect roi1, roi2;
 const int margin = 75;
 if(counter1 < 5)
 {
 int xLeft = lastBegin1.pt.x - margin;
 int xRight = lastEnd1.pt.x + margin;
 int yTop = lastBegin1.pt.y - margin;
 int yBottom = lastEnd1.pt.y + margin;
 
 if(xLeft > lastEnd1.pt.x - margin)
 {
 xLeft = lastEnd1.pt.x - margin;
 }
 if(xLeft < 0)
 {
 xLeft = 0;
 }
 
 if(xRight < lastBegin1.pt.x + margin)
 {
 xRight = lastBegin1.pt.x + margin;
 }
 if(xRight > src1.cols - 1)
 {
 xRight = src1.cols - 1;
 }
 
 if(yTop > lastEnd1.pt.y - margin)
 {
 yTop = lastEnd1.pt.y - margin;
 }
 if(yTop < 0)
 {
 yTop = 0;
 }
 
 if(yBottom < lastBegin1.pt.y + margin)
 {
 yBottom = lastBegin1.pt.y + margin;
 }
 if(yBottom > src1.rows - 1)
 {
 yBottom = src1.rows - 1;
 }
 
 roi1 = Rect(xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1);
 Mat roiMat = src1(roi1);
 //imshow("ROI1", roiMat);
 
 detected1 = GetStrip(roiMat, begin1, end1, thresh1);
 begin1.pt.x += roi1.x;
 end1.pt.x += roi1.x;
 begin1.pt.y += roi1.y;
 end1.pt.y += roi1.y;
 
 if(detected1)
 {
 counter1 = 0;
 lastBegin1 = begin1;
 lastEnd1 = end1;
 }
 else
 {
 counter1++;
 }
 }
 else
 {
 detected1 = GetStrip(src1, begin1, end1, thresh1);
 if(detected1)
 {
 counter1 = 0;
 lastBegin1 = begin1;
 lastEnd1 = end1;
 }
 }
 if(counter2 < 5)
 {
 int xLeft = lastBegin2.pt.x - margin;
 int xRight = lastEnd2.pt.x + margin;
 int yTop = lastBegin2.pt.y - margin;
 int yBottom = lastEnd2.pt.y + margin;
 
 if(xLeft > lastEnd2.pt.x - margin)
 {
 xLeft = lastEnd2.pt.x - margin;
 }
 if(xLeft < 0)
 {
 xLeft = 0;
 }
 
 if(xRight < lastBegin2.pt.x + margin)
 {
 xRight = lastBegin2.pt.x + margin;
 }
 if(xRight > src2.cols - 1)
 {
 xRight = src2.cols - 1;
 }
 
 if(yTop > lastEnd2.pt.y - margin)
 {
 yTop = lastEnd2.pt.y - margin;
 }
 if(yTop < 0)
 {
 yTop = 0;
 }
 
 if(yBottom < lastBegin2.pt.y + margin)
 {
 yBottom = lastBegin2.pt.y + margin;
 }
 if(yBottom > src2.rows - 1)
 {
 yBottom = src2.rows - 1;
 }
 
 roi2 = Rect(xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1);
 Mat roiMat = src2(roi2);
 //imshow("ROI2", roiMat);
 
 detected2 = GetStrip(roiMat, begin2, end2, thresh2);
 begin2.pt.x += roi2.x;
 end2.pt.x += roi2.x;
 begin2.pt.y += roi2.y;
 end2.pt.y += roi2.y;
 if(detected2)
 {
 counter2 = 0;
 lastBegin2 = begin2;
 lastEnd2 = end2;
 }
 else
 {
 counter2++;
 }
 }
 else
 {
 detected2 = GetStrip(src2, begin2, end2, thresh2);
 if(detected2)
 {
 counter2 = 0;
 lastBegin2 = begin2;
 lastEnd2 = end2;
 }
 }
 
 if(!detected1 || !detected2)
 {
 begin1 = lastBegin1;
 end1 = lastEnd1;
 begin2 = lastBegin2;
 end2 = lastEnd2;
 }
 //imshow("Thresh1", thresh1);
 //imshow("Thresh2", thresh2);
 return detected1 && detected2;
 }
 
 /// GetStrip
 /// This function was originally used when each link was equipped with strips of colour
 /// to calculate the vector and hence calculate the joint positions.
 /// However it was replaced by the GetJointBlob function for its reliability and robustness.
 bool BlobHueDetector::GetStrip(Mat &src, KeyPoint &begin, KeyPoint &end, Mat& thresh)
 {
 Mat hsv;
 cvtColor(src, hsv, CV_BGR2HSV);
 
 if(iLowH > iHighH)
 {
 Mat temp1, temp2;
 inRange(hsv, Scalar(0, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), temp1);
 inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(179, iHighS, iHighV), temp2);
 
 bitwise_or(temp1, temp2, thresh);
 }
 else
 {
 inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh);
 }    thresh = 255 - thresh;
 
 int erosionSize = 3;
 int dilationSize = 3;
 
 ApplyMorphologicalOperation(thresh, erosionSize, dilationSize);
 
 vector<KeyPoint> keypoints;
 detector.detect(thresh, keypoints);
 
 
 int i = 0, j = 0, k = 0;
 double blobSize = 0, blobSize2 = 0;
 if(keypoints.size() < 2)
 {
 return false;
 }
 else
 {
 for(vector<KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
 
 if(blobIterator->size > blobSize)
 {
 
 blobSize2 = blobSize;
 j = k;
 blobSize = blobIterator->size;
 k = i;
 }
 else if(blobIterator->size > blobSize2)
 {
 blobSize2 = blobIterator->size;
 j = i;
 }
 i++;
 }
 begin = keypoints.at(k);
 end = keypoints.at(j);
 return true;
 }
 }
 */

//-----------------------MovingAverageFilter-----------------------------//

/// MovingAverageFilter constructor
BlobHueDetector::MovingAverageFilter::MovingAverageFilter()
{
    Init();
}

/// Add
/// In: p: point to add to list
/// Out: new average point
Point2f BlobHueDetector::MovingAverageFilter::Add(Point2f p)
{
    // add point to list and increment iterator
    pointArray[iterator] = p;
    iterator++;
    
    // loop around when iterator reaches max count
    if(iterator == FILTER_NUMBER)
    {
        iterator = 0;
    }
    
    // calculate average
    Point2f sum = Point2f(0, 0);
    for (int i = 0; i < FILTER_NUMBER; i++)
    {
        sum += pointArray[i];
    }
    return sum * ((float)1/FILTER_NUMBER);
}

/// Average
/// Out: average point
Point2f BlobHueDetector::MovingAverageFilter::Average()
{
    // calculate average
    Point2f sum = Point2f(0, 0);
    for (int i = 0; i < FILTER_NUMBER; i++){
        sum += pointArray[i];
    }
    return sum * ((float)1/FILTER_NUMBER);
}

/// Init
void BlobHueDetector::MovingAverageFilter::Init()
{
    // initialise list
    for (int i = 0; i < FILTER_NUMBER; i++)
    {
        pointArray[i] = Point2f(0, 0);
    }
    iterator = 0;
}
