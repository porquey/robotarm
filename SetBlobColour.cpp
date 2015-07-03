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


void SetBlobColour(VideoCapture& inputCapture)
{
    namedWindow("Src", WINDOW_AUTOSIZE);
    Mat src;
    //inputCapture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    //inputCapture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    while(inputCapture.isOpened())
    {
        inputCapture.read(src);
        imshow("Src", src);

        Mat hsv, thresh;
        cvtColor(src, hsv, CV_BGR2HSV);
        void* userData = static_cast<void*>(&hsv);
        setMouseCallback("Src", MouseCallBack, userData);
        
        inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh);
        thresh = 255 - thresh;
        Mat erosionElement = getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(2 * 3 + 1, 2 * 3 + 1),
                                                   cv::Point(3, 3) );
        
        // Apply erosion or dilation on the image
        erode(thresh, thresh, erosionElement);
        
        Mat dilationElement = getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(2 * 4 + 1, 2 * 4 + 1),
                                                    cv::Point(4, 4) );
        
        // Apply erosion or dilation on the image
        dilate(thresh, thresh, dilationElement);
        
        imshow("Thresh", thresh);
        
        int key = waitKey(30);
        
        if((char)key == 'q')
        {
            iHighH += 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
            
        }
        else if((char)key == 'w')
        {
            iLowH += 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'e')
        {
            iHighS += 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'r')
        {
            iLowS += 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 't')
        {
            iHighV += 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'y')
        {
            iLowV += 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }else if((char)key == 'a')
        {
            iHighH -= 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
            
        }
        else if((char)key == 's')
        {
            iLowH -= 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'd')
        {
            iHighS -= 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'f')
        {
            iLowS -= 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'g')
        {
            iHighV -= 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == 'h')
        {
            iLowV -= 5;
            cout << "H: " << iHighH << " - " << iLowH << endl;
            cout << "S: " << iHighS << " - " << iLowS << endl;
            cout << "V: " << iHighV << " - " << iLowV << endl;
        }
        else if((char)key == ' ')
        {
            printf("Saving...\n");
            const string fileName = "BlobHSVColour.xml";
            FileStorage fs(fileName, FileStorage::WRITE);
            Mat hsvData = (Mat_<int> (3,2) << iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
            fs << "HSV_Data" << hsvData;
            return;
        }
            
    }
}
