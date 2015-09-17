  #include "SetBlobColour.h"


void MouseCallBack(int event, int x, int y, int flags, void* userData)
{
    
    if(event == EVENT_LBUTTONDOWN)
    {
        Mat* img = static_cast<Mat*> (userData);
        cerr << "X: " << x << " Y:" << y << endl;
        Vec3b pixel = img->at<Vec3b>(y, x);
        cerr << "H: " << (int)pixel[0] << " S: " << (int)pixel[1] << " V: " << (int)pixel[2] << endl;
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
        if(iLowH > iHighH)
        {
            Mat temp1, temp2, temp3, temp4;
            inRange(hsv1, Scalar(0, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), temp1);
            inRange(hsv1, Scalar(iLowH, iLowS, iLowV), Scalar(179, iHighS, iHighV), temp2);
            inRange(hsv2, Scalar(0, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), temp3);
            inRange(hsv2, Scalar(iLowH, iLowS, iLowV), Scalar(179, iHighS, iHighV), temp4);

            bitwise_or(temp1, temp2, thresh1);
            bitwise_or(temp3, temp4, thresh2);
            
        }
        else
        {
            inRange(hsv1, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh1);
            inRange(hsv2, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh2);
        }
        thresh1 = 255 - thresh1;
        thresh2 = 255 - thresh2;

        
        ApplyMorphologicalOperation(thresh1, erosion, dilation);
        ApplyMorphologicalOperation(thresh2, erosion, dilation);
        
        imshow("Thresh1", thresh1);
        imshow("Thresh2", thresh2);
        
        int key = waitKey(30);
        
        if((char)key == 't')
        {
            iHighH += 2;
            if(iHighH > 179)
            {
                iHighH = iHighH - 180;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
            
        }
        else if((char)key == 'y')
        {
            iLowH += 2;
            if(iLowH > 179)
            {
                iLowH = iLowH - 180;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'u')
        {
            iHighS += 2;
            if(iHighS > 255)
            {
                iHighS = 255;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'i')
        {
            iLowS += 2;
            if(iLowS > 255)
            {
                iLowS = 255;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'o')
        {
            iHighV += 2;
            if(iHighV > 255)
            {
                iHighV = 255;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'p')
        {
            iLowV += 2;
            if(iLowV > 255)
            {
                iLowV = 255;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }else if((char)key == 'g')
        {
            iHighH -= 2;
            if(iHighH < 0)
            {
                iHighH = iHighH + 180;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
            
        }
        else if((char)key == 'h')
        {
            iLowH -= 2;
            if(iLowH < 0)
            {
                iLowH = iLowH + 180;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'j')
        {
            iHighS -= 2;
            if(iHighS < 0)
            {
                iHighS = 0;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'k')
        {
            iLowS -= 2;
            if(iLowS < 0)
            {
                iLowS = 0;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'l')
        {
            iHighV -= 2;
            if(iHighV < 0)
            {
                iHighV = 0;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == ';')
        {
            iLowV -= 2;
            if(iLowV < 0)
            {
                iLowV = 0;
            }
            cerr << "H: " << iHighH << " - " << iLowH << endl;
            cerr << "S: " << iHighS << " - " << iLowS << endl;
            cerr << "V: " << iHighV << " - " << iLowV << endl;
        }
        
        else if((char)key == '2')
        {
            erosion++;
            cerr << "Erosion: " << erosion << endl;
        }
        else if((char)key == '1')
        {
            if(erosion != 1)
            {
                erosion--;
            }
            cerr << "Erosion: " << erosion << endl;
        }
        else if((char)key == '4')
        {
            dilation++;
            cerr << "Dilation: " << dilation << endl;
        }
        else if((char)key == '3')
        {
            if(dilation != 1)
            {
                dilation--;
            }
            cerr << "Dilation: " << dilation << endl;
        }
        else if((char)key == 's')
        {
            cerr << "Saving..." << endl;
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
            cerr << "Target saved..." << endl;
            
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
            cout << "Done..." << endl;
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
            cout << "Cancelling..." << endl;
            return;
        }

    }
}

void ApplyMorphologicalOperation(Mat &img, int erosionSize, int dilationSize)
{
    
    Mat erosionElement = getStructuringElement(cv::MORPH_ELLIPSE, Size(erosionSize, erosionSize), Point(-1, -1));
    
    // Apply erosion or dilation on the image
    
    Mat dilationElement = getStructuringElement(MORPH_ELLIPSE, Size(dilationSize, dilationSize), Point(-1, -1));
    
    Mat erosionElement2 = getStructuringElement(cv::MORPH_ELLIPSE, Size(2 * erosionSize, 2 * erosionSize), Point(-1, -1));
    
    // Apply erosion or dilation on the image
    
    Mat dilationElement2 = getStructuringElement(MORPH_ELLIPSE, Size(2 * dilationSize, 2 * dilationSize), Point(-1, -1));
    
    erode(img, img, erosionElement2);
    
    dilate(img, img, dilationElement);
    
    erode(img, img, erosionElement);

    // Apply erosion or dilation on the image
    dilate(img, img, dilationElement);
    
    // Apply erosion or dilation on the image
    //erode(img, img, erosionElement);
    
}
