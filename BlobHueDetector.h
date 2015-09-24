#include "opencv2/opencv.hpp"
#include "BetterBlobDetector.h"
#include "HSVRange.h"
#include "SetBlobColour.h"
#include <iostream>

#define FILTER_NUMBER 3

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace std;
using namespace cv;

/// BlobHueDetector class allows the detection of coloured markers within distinct HSV ranges
class BlobHueDetector
{
public:
    /// MovingAverageFilter class calculates the average of a point over 3 frames
    class MovingAverageFilter
    {
    public:
        // constructor
        MovingAverageFilter();
        // add point to list
        Point2f Add(Point2f p);
        // get average of points
        Point2f Average();
        // initialise moving average filter
        void Init();
        
    private:
        // class members
        Point2f pointArray[FILTER_NUMBER];
        int iterator;
    };
    
public:
    // constructor
    BlobHueDetector();
    // get centre points of colour blobs from a set of images
    bool GetBlobCentres(Mat &src1, Mat &src2, KeyPoint &keypoint1, KeyPoint &keypoint2);
    // get centre point of colour blob from an image
    bool GetBlob(Mat &src, KeyPoint &keypoint);
    // get centre points of colour markers on a joint from a set of images
    bool GetJointPos(vector<Mat> &srcVec, vector<KeyPoint> &pointVec);
    // get centre point of colour markers on a joint
    bool GetJointBlob(Mat &src, KeyPoint &point, Mat &thresh);
    // set the HSV ranges for detection
    void SetHSVRanges(const HSVRanges range);
    // set the default HSV ranges
    void SetDefaultHSVRanges();
    
    //bool GetStripVectors(Mat &src1, Mat &src2, KeyPoint &begin1, KeyPoint &end1, KeyPoint &begin2, KeyPoint &end2);
    //bool GetStrip(Mat &src, KeyPoint &begin, KeyPoint &end, Mat &thresh);
    
private:
    // class members
    
    // blob detection and parameters
    BetterBlobDetector detector;
    SimpleBlobDetector::Params params;
    
    // HSV ranges
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    
    // region of interest information
    int counter1, counter2;
    KeyPoint last1, last2;
    KeyPoint lastBegin1, lastEnd1, lastBegin2, lastEnd2;
    vector<int> countVec;
    vector<KeyPoint> offsetVec;
    
public:
    // moving average filter
    MovingAverageFilter filter[2];
};
