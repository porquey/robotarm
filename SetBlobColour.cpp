#include "SetBlobColour.h"


void MouseCallBack(int event, int x, int y, int flags, void* userData)
{
    
    if(event == EVENT_LBUTTONDOWN)
    {
        Mat* img = static_cast<Mat*> (userData);
        cout << "X: " << x << " Y:" << y << endl;
        Vec3b pixel = img->at<Vec3b>(y, x);
        cout << "H: " << (int)pixel[0] << " S: " << (int)pixel[1] << " V: " << (int)pixel[2] << endl;
        iLowH = (int)pixel[0] - 10;
        iHighH = (int)pixel[0] + 10;
        iLowS = (int)pixel[1] - 80;
        iHighS = (int)pixel[1] + 80;
        iLowV = (int)pixel[2] - 70;
        iHighV = (int)pixel[2] + 70;
    }
}

void SetBlobColour(VideoCapture& inputCapture1, VideoCapture& inputCapture2)
{
    namedWindow("Src1", WINDOW_AUTOSIZE);
    namedWindow("Src2", WINDOW_AUTOSIZE);

    Mat src1, src2;

    vector<HSVRanges> hsvVector;
    HSVRanges target;
    
    while(inputCapture1.isOpened())
    {
        inputCapture1.read(src1);
        inputCapture2.read(src2);
        imshow("Src1", src1);
        imshow("Src2", src2);

        Mat hsv1, hsv2, thresh1, thresh2;
        
        cvtColor(src1, hsv1, CV_BGR2HSV);
        cvtColor(src2, hsv2, CV_BGR2HSV);
        
        void* userData1 = static_cast<void*>(&hsv1);
        void* userData2 = static_cast<void*>(&hsv2);

        setMouseCallback("Src1", MouseCallBack, userData1);
        setMouseCallback("Src2", MouseCallBack, userData2);

        inRange(hsv1, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh1);
        inRange(hsv2, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh2);
        
        thresh1 = 255 - thresh1;
        thresh2 = 255 - thresh2;

        Mat erosionElement = getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(2 * 3 + 1, 2 * 3 + 1),
                                                   cv::Point(3, 3) );
        
        // Apply erosion or dilation on the image
        erode(thresh1, thresh1, erosionElement);
        erode(thresh2, thresh2, erosionElement);
        
        Mat dilationElement = getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(2 * 4 + 1, 2 * 4 + 1),
                                                    cv::Point(4, 4) );
        
        // Apply erosion or dilation on the image
        dilate(thresh1, thresh1, dilationElement);
        dilate(thresh2, thresh2, dilationElement);
        
        imshow("Thresh1", thresh1);
        imshow("Thresh2", thresh2);
        
        int key = waitKey(30);
        
        if((char)key == 't')
        {
            iHighH += 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
            
        }
        else if((char)key == 'y')
        {
            iLowH += 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'u')
        {
            iHighS += 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'i')
        {
            iLowS += 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'o')
        {
            iHighV += 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'p')
        {
            iLowV += 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }else if((char)key == 'g')
        {
            iHighH -= 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
            
        }
        else if((char)key == 'h')
        {
            iLowH -= 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'j')
        {
            iHighS -= 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'k')
        {
            iLowS -= 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'l')
        {
            iHighV -= 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == ';')
        {
            iLowV -= 2;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 's')
        {
            printf("Saving...\n");
            HSVRanges ranges;

            ranges.lowH = iLowH;
            ranges.highH = iHighH;
            ranges.lowS = iLowS;
            ranges.highS = iHighS;
            ranges.lowV = iLowV;
            ranges.highV = iHighV;
            
            hsvVector.push_back(ranges);
            
            iLowH = 0;
            iHighH = 179;
            iLowS = 0;
            iHighS = 255;
            iLowV = 0;
            iHighV = 255;
        }
        else if((char)key == 'a')
        {
            printf("Target saved...\n");
            
            target.lowH = iLowH;
            target.highH = iHighH;
            target.lowS = iLowS;
            target.highS = iHighS;
            target.lowV = iLowV;
            target.highV = iHighV;
            
            iLowH = 0;
            iHighH = 179;
            iLowS = 0;
            iHighS = 255;
            iLowV = 0;
            iHighV = 255;
        }
        else if((char)key == 'd')
        {
            printf("Done...\n");
            const string fileName = "BlobHSVColour.xml";
            FileStorage fs(fileName, FileStorage::WRITE);
            
            Mat hsvSize = (Mat_<int> (1,1) << hsvVector.size());
            fs << "HSV_Size" << hsvSize;
            
            for(int i = 0; i < hsvVector.size(); i++)
            {
                Mat hsvData = (Mat_<int> (3,2) << hsvVector[i].lowH, hsvVector[i].highH, hsvVector[i].lowS, hsvVector[i].highS, hsvVector[i].lowV, hsvVector[i].highV);
                string dataName = "HSV_Data_" + to_string(i);
                fs << dataName << hsvData;
            }
            Mat targetData = (Mat_<int> (3,2) << target.lowH, target.highH, target.lowS, target.highS, target.lowV, target.highV);
            fs << "Target_Data" << targetData;
            return;
        }
        else if((char)key == 'c')
        {
            printf("Cancelling...\n");
            return;
        }

    }
}
