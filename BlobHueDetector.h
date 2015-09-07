#include "opencv2/opencv.hpp"
#include "BetterBlobDetector.h"
#include "HSVRange.h"
#include "SetBlobColour.h"
#include <iostream>

#define FILTER_NUMBER 5

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

class BlobHueDetector
{
public:
    class MovingAverageFilter
    {
    public:
        MovingAverageFilter();
        cv::Point2f Add(cv::Point2f p);
        cv::Point2f Average();
        void Init();
    private:
        cv::Point2f pointArray[FILTER_NUMBER];
        int iterator;
    };
    
public:
    BlobHueDetector();
    
    bool GetBlobCentres(cv::Mat &src1, cv::Mat &src2, cv::KeyPoint &keypoint1, cv::KeyPoint &keypoint2);
    bool GetBlob(cv::Mat &src, cv::KeyPoint &keypoint);
    bool GetStripVectors(cv::Mat &src1, cv::Mat &src2, cv::KeyPoint &begin1, cv::KeyPoint &end1, cv::KeyPoint &begin2, cv::KeyPoint &end2);
    bool GetStrip(cv::Mat &src, cv::KeyPoint &begin, cv::KeyPoint &end, cv::Mat &thresh);
    bool GetJointPos(std::vector<cv::Mat> &srcVec, std::vector<cv::KeyPoint> &pointVec);
    bool GetJointBlob(cv::Mat &src, cv::KeyPoint &point, cv::Mat &thresh);
    
    void SetHSVRanges(const HSVRanges range);
    void SetDefaultHSVRanges();
private:
    BetterBlobDetector detector;
    cv::SimpleBlobDetector::Params params;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    int counter1, counter2;
    cv::KeyPoint last1, last2;
    
    cv::KeyPoint lastBegin1, lastEnd1, lastBegin2, lastEnd2;
    
    std::vector<int> countVec;
    std::vector<cv::KeyPoint> offsetVec;
public:
    MovingAverageFilter filter[2];
};
