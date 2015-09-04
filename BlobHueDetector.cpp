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
last1(),
last2(),
lastBegin1(),
lastEnd1(),
lastBegin2(),
lastEnd2(),
countVec(),
offsetVec()
{
    params.minThreshold = 80;
    params.maxThreshold = 150;
    
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;
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
    
    params.minDistBetweenBlobs = 50;
    
    detector = BetterBlobDetector(params);
    
    countVec.push_back(5);
    countVec.push_back(5);
    
    offsetVec.push_back(cv::KeyPoint());
    offsetVec.push_back(cv::KeyPoint());
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
        int x = last1.pt.x - 50;
        int y = last1.pt.y - 50;
        int width = 100;
        int height = 100;

        if(last1.pt.x - 50 < 0)
        {
            x = 0;
        }
        if(last1.pt.y - 50 < 0)
        {
            y = 0;
        }
        if(last1.pt.x + 50 > src1.cols)
        {
            width = src1.cols - last1.pt.x + 50;
        }
        if(last1.pt.y + 50 > src1.rows)
        {
            height = src1.rows - last1.pt.y + 50;
        }
        roi1 = cv::Rect(x, y, width, height);
        cv::Mat roiMat = src1(roi1);
        detected1 = GetBlob(roiMat, keypoint1);
        keypoint1.pt.x += roi1.x;
        keypoint1.pt.y += roi1.y;
        if(detected1)
        {
            counter1 = 0;
            last1 = keypoint1;
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
            last1 = keypoint1;
        }
    }
    
    if(counter2 < 5)
    {
        int x = last2.pt.x - 50;
        int y = last2.pt.y - 50;
        int width = 100;
        int height = 100;
        
        if(last2.pt.x - 50 < 0)
        {
            x = 0;
        }
        if(last2.pt.y - 50 < 0)
        {
            y = 0;
        }
        if(last2.pt.x + 50 > src2.cols)
        {
            width = src2.cols - last2.pt.x + 50;
        }
        if(last2.pt.y + 50 > src2.rows)
        {
            height = src2.rows - last2.pt.y + 50;
        }
        roi2 = cv::Rect(x, y, width, height);
        cv::Mat roiMat = src2(roi2);
        detected2 = GetBlob(roiMat, keypoint2);
        keypoint2.pt.x += roi2.x;
        keypoint2.pt.y += roi2.y;
        if(detected2)
        {
            counter2 = 0;
            last2 = keypoint2;
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
            last2 = keypoint2;
        }
    }
    if(!detected1 || !detected2)
    {
        keypoint1 = last1;
        keypoint2 = last2;
    }
    return detected1 && detected2;
}

bool BlobHueDetector::GetBlob(cv::Mat &src, cv::KeyPoint &keypoint)
{
    cv::Mat hsv, thresh;
    cv::cvtColor(src, hsv, CV_BGR2HSV);
    
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
    double blobSize = 0;
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

bool BlobHueDetector::GetStripVectors(cv::Mat &src1, cv::Mat &src2, cv::KeyPoint &begin1, cv::KeyPoint &end1, cv::KeyPoint &begin2, cv::KeyPoint &end2)
{
    bool detected1 = false, detected2 = false;
    cv::Mat thresh1, thresh2;
    cv::Rect roi1, roi2;
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
        
        roi1 = cv::Rect(xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1);
        cv::Mat roiMat = src1(roi1);
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
        
        roi2 = cv::Rect(xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1);
        cv::Mat roiMat = src2(roi2);
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
    imshow("Thresh1", thresh1);
    imshow("Thresh2", thresh2);
    return detected1 && detected2;
}

bool BlobHueDetector::GetStrip(cv::Mat &src, cv::KeyPoint &begin, cv::KeyPoint &end, Mat& thresh)
{
    cv::Mat hsv;
    cv::cvtColor(src, hsv, CV_BGR2HSV);
    
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

    std::vector<cv::KeyPoint> keypoints;
    detector.detect(thresh, keypoints);
    
    
    int i = 0, j = 0, k = 0;
    double blobSize = 0, blobSize2 = 0;
    if(keypoints.size() < 2)
    {
        return false;
    }
    else
    {
        for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){

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

bool BlobHueDetector::GetJointPos(std::vector<cv::Mat> &srcVec, std::vector<cv::KeyPoint> &pointVec)
{
    cv::Mat thresh;
    cv::Rect roi;
    const int margin = 75;
    std::vector<bool> detectedVec;
    for(int i = 0; i < srcVec.size(); i++)
    {
        if(countVec[i] < 5)
        {
            int xLeft = offsetVec[i].pt.x - margin;
            int xRight = offsetVec[i].pt.x + margin;
            int yTop = offsetVec[i].pt.y - margin;
            int yBottom = offsetVec[i].pt.y + margin;
            
            if(xLeft > offsetVec[i].pt.x - margin)
            {
                xLeft = offsetVec[i].pt.x - margin;
            }
            if(xLeft < 0)
            {
                xLeft = 0;
            }
            
            if(xRight < offsetVec[i].pt.x + margin)
            {
                xRight = offsetVec[i].pt.x + margin;
            }
            if(xRight > srcVec[i].cols - 1)
            {
                xRight = srcVec[i].cols - 1;
            }
            
            if(yTop > offsetVec[i].pt.y - margin)
            {
                yTop = offsetVec[i].pt.y - margin;
            }
            if(yTop < 0)
            {
                yTop = 0;
            }
            
            if(yBottom < offsetVec[i].pt.y + margin)
            {
                yBottom = offsetVec[i].pt.y + margin;
            }
            if(yBottom > srcVec[i].rows - 1)
            {
                yBottom = srcVec[i].rows - 1;
            }
            
            roi = cv::Rect(xLeft, yTop, xRight - xLeft + 1, yBottom - yTop + 1);
            cv::Mat roiMat = srcVec[i](roi);
            //imshow("ROI1", roiMat);
            cv::KeyPoint point;
            detectedVec.push_back(GetJointBlob(roiMat, point, thresh));
            point.pt.x += roi.x;
            point.pt.y += roi.y;
            pointVec.push_back(point);
            if(detectedVec[i])
            {
                countVec[i] = 0;
                offsetVec[i] = point;
            }
            else
            {
                countVec[i]++;
            }
        }
        else
        {
            cv::KeyPoint point;
            detectedVec.push_back(GetJointBlob(srcVec[i], point, thresh));
            pointVec.push_back(point);
            if(detectedVec[i])
            {
                countVec[i] = 0;
                offsetVec[i] = point;
            }
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

bool BlobHueDetector::GetJointBlob(cv::Mat &src, cv::KeyPoint &point, cv::Mat &thresh)
{
    cv::Mat hsv;
    cv::cvtColor(src, hsv, CV_BGR2HSV);
    
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
    
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(thresh, keypoints);
    
    
    int k = 0;
    double blobSize = 0;
    if(keypoints.size() == 0)
    {
        return false;
    }
    else
    {
        for(int i = 0; i < keypoints.size(); i++){
            
            if(keypoints[i].size > blobSize)
            {
                blobSize = keypoints[i].size;
                k = i;
            }
        }
        std::vector<cv::KeyPoint> cluster;
        
        cluster.push_back(keypoints.at(k));
        
        for(int i = 0; i < keypoints.size(); i++)
        {
            if(i == k)
            {
                //cerr << "Detection point: " << keypoints[i].pt << endl;
                //cv::circle(src, keypoints[i].pt, 5, Scalar(255,255,255));
            }
            else
            {
                double distance = sqrt((keypoints[i].pt.x - keypoints[k].pt.x) * (keypoints[i].pt.x - keypoints[k].pt.x) + (keypoints[i].pt.y - keypoints[k].pt.y) * (keypoints[i].pt.y - keypoints[k].pt.y));
                //cerr << "Point " << i << " is: " << keypoints[i].pt << " dist: " << distance << endl;
                //cv::circle(src, keypoints[i].pt, 5, Scalar(0,255,255));
                if(distance < 60)
                {
                    cluster.push_back(keypoints[i]);
                }
            }
        
        }
        cv::Point2f avg = cv::Point2f(0, 0);
        double sum = 0;

        for(int i = 0; i < cluster.size(); i++)
        {
            avg = avg + cluster[i].pt * cluster[i].size;
            sum += cluster[i].size;
        }
        double denom = 1/sum;
        avg = avg * denom;
        //cerr << "Point " << avg << " size " << sum << endl;
        
        point = cv::KeyPoint(avg, sum);

        cv::imshow("TH", thresh);
        return true;
    }

    
}

