#include "CalibrateEnvironment.h"


/// Calculates the positions of the corners relative to the top left corner
/// In: boardSize: size of the board (width and height)
///     squareSize: size of the chessboard squares
/// Out: corners: corner locations in x and y positions
void CalcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.clear();
    
    for( int i = 0; i < boardSize.height; ++i )
        for( int j = 0; j < boardSize.width; ++j )
            corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
}

/// Calculates the corner pixel locations as detected by each camera
/// In: s: board settings, includes size, square size and the number of corners
///     inputCapture1: video capture for camera 1
///     inputCapture2: video capture for camera 2
///     iterations: number of chessboard images to take
/// Out: imagePoints1: pixel coordinates of chessboard corners for camera 1
///      imagePoints2: pixel coordinates of chessboard corners for camera 2
bool RetrieveChessboardCorners(vector<vector<Point2f> >& imagePoints1, vector<vector<Point2f> >& imagePoints2, BoardSettings s, VideoCapture inputCapture1,VideoCapture inputCapture2, int iterations)
{
    destroyAllWindows();
    Mat image1,image2;
    vector<Point2f> pointBuffer1;
    vector<Point2f> pointBuffer2;
    clock_t prevTimeStamp = 0;
    bool found1,found2;
    int count = 0;
    while (count != iterations){
        char c = waitKey(15);
        if (c == 's'){
            cerr << "Calibration stopped" << endl;
            return false;
        }
        // try find chessboard corners
        else if(c == 'c'){
            // ADAPTIVE_THRESH -> use adaptive thresholding to convert image to B&W
            // FAST_CHECK -> Terminates call earlier if no chessboard in image
            // NORMALIZE_IMAGE -> normalize image gamma before thresholding
            // FILTER_QUADS -> uses additional criteria to filter out false quads
            found1 = findChessboardCorners(image1, s.boardSize, pointBuffer1,
                                           CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
                                           CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);
            found2 = findChessboardCorners(image2, s.boardSize, pointBuffer2,
                                           CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
                                           CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);
            
            if (found1 && found2 && (pointBuffer1.size() >= s.cornerNum) && (pointBuffer2.size() >= s.cornerNum)){
                // if time delay passed refine accuracy and store
                if ((clock() - prevTimeStamp) > CAPTURE_DELAY * 1e-3*CLOCKS_PER_SEC){
                    Mat imageGray1, imageGray2;
                    cvtColor(image1, imageGray1, COLOR_BGR2GRAY);
                    cvtColor(image2, imageGray2, COLOR_BGR2GRAY);
                    
                    // refines corner locations
                    // Size(11,11) -> size of the search window
                    // Size(-1,-1) -> indicates no dead zone in search size
                    // TermCriteria -> max 1000 iteration, to get acuraccy of 0.01
                    cornerSubPix(imageGray1, pointBuffer1, Size(5,5), Size(-1, -1),
                                 TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 1000, 0.01));
                    cornerSubPix(imageGray2, pointBuffer2, Size(5,5), Size(-1, -1),
                                 TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 1000, 0.01));
                    
                    drawChessboardCorners(image1, s.boardSize, Mat(pointBuffer1), found1);
                    imshow("Image View1", image1);
                    drawChessboardCorners(image2, s.boardSize, Mat(pointBuffer2), found2);
                    imshow("Image View2", image2);
                    
                    // user verifies the correct corners have been found
                    c = waitKey(0);
                    if (c == 's'){
                        return false;
                    }
                    if (c == 'y'){
                        // store the points and store time stamp
                        imagePoints1.push_back(pointBuffer1);
                        imagePoints2.push_back(pointBuffer2);
                        prevTimeStamp = clock();
                        count++;
                        cerr << "Count: " << count << endl;
                    }
                }
            }
        }
        inputCapture1.read(image1);
        inputCapture2.read(image2);
        imshow("Image View1", image1);
        imshow("Image View2", image2);
    }
    // found all corners
    return true;
}

/// Calibrates the extrinsic parameters of the setup and saves it to an XML file
/// Press'r' to retreive chessboard corners
///      's' to save and exit
///      'c' to exit without saving
/// In: inputCapture1: video feed of camera 1
///     inputCapture2: video feed of camera 2
void CalibrateEnvironment(VideoCapture& inputCapture1, VideoCapture& inputCapture2)
{
    Size boardSize;
    boardSize.width = BOARD_WIDTH;
    boardSize.height = BOARD_HEIGHT;
    
    const string fileName1 = "CameraIntrinsics1.xml";
    const string fileName2 = "CameraIntrinsics2.xml";
    
    cerr << "Attempting to open configuration files" << endl;
    FileStorage fs1(fileName1, FileStorage::READ);
    FileStorage fs2(fileName2, FileStorage::READ);
    
    Mat cameraMatrix1, cameraMatrix2;
    Mat distCoeffs1, distCoeffs2;
    
    fs1["Camera_Matrix"] >> cameraMatrix1;
    fs1["Distortion_Coefficients"] >> distCoeffs1;
    fs2["Camera_Matrix"] >> cameraMatrix2;
    fs2["Distortion_Coefficients"] >> distCoeffs2;
    
    if (cameraMatrix1.data == NULL || distCoeffs1.data == NULL ||
        cameraMatrix2.data == NULL || distCoeffs2.data == NULL)
    {
        cerr << "Could not load camera intrinsics\n" << endl;
    }
    else{
        cerr << "Loaded intrinsics\n" << endl;
        cerr << "Camera Matrix1: " << cameraMatrix1 << endl;
        cerr << "Camera Matrix2: " << cameraMatrix2 << endl;
        
    }
    
    Mat translation;
    Mat image1, image2;
    Mat mapX1, mapX2, mapY1, mapY2;
    inputCapture1.read(image1);
    Size imageSize = image1.size();
    bool rotationCalibrated = false;
    
    while(inputCapture1.isOpened() && inputCapture2.isOpened())
    {
        inputCapture1.read(image1);
        inputCapture2.read(image2);
        
        if (rotationCalibrated)
        {
            Mat t1 = image1.clone();
            Mat t2 = image2.clone();
            remap(t1, image1, mapX1, mapY1, INTER_LINEAR);
            remap(t2, image2, mapX2, mapY2, INTER_LINEAR);
            t1.release();
            t2.release();
        }
        
        char c = waitKey(15);
        if (c == 'c')
        {
            cerr << "Cancelling..." << endl;
            return;
        }
        else if(c == 's' && rotationCalibrated)
        {
            cerr << "Saving..." << endl;
            const string fileName = "EnvironmentCalibration.xml";
            FileStorage fs(fileName, FileStorage::WRITE);
            fs << "Camera_Matrix_1" <<  getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, imageSize, 1,imageSize, 0);
            fs << "Camera_Matrix_2" <<  getOptimalNewCameraMatrix(cameraMatrix2, distCoeffs2, imageSize, 1, imageSize, 0);
            fs << "Mapping_X_1" << mapX1;
            fs << "Mapping_Y_1" << mapY1;
            fs << "Mapping_X_2" << mapX2;
            fs << "Mapping_Y_2" << mapY2;
            fs << "Translation" << translation;
            cerr << "Exiting..." << endl;
            destroyAllWindows();
            return;
        }
        else if(c == 's' && !rotationCalibrated)
        {
            cerr << "Exiting..." << endl;
            destroyAllWindows();
            return;
        }
        else if (c == 'r')
        {
            BoardSettings s;
            s.boardSize.width = BOARD_WIDTH;
            s.boardSize.height = BOARD_HEIGHT;
            s.cornerNum = s.boardSize.width * s.boardSize.height;
            s.squareSize = (float)SQUARE_SIZE;
            
            vector<Point3f> objectPoints;
            vector<vector<Point2f> > imagePoints1, imagePoints2;
            
            if (RetrieveChessboardCorners(imagePoints1, imagePoints2, s, inputCapture1, inputCapture2, ITERATIONS))
            {
                vector<vector<Point3f> > objectPoints(1);
                CalcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0]);
                objectPoints.resize(imagePoints1.size(),objectPoints[0]);
                
                Mat R, T, E, F;
                Mat rmat1, rmat2, rvec;
                
                double rms = stereoCalibrate(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, E, F,
                                             TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 1000, 0.01),
                                             CV_CALIB_FIX_INTRINSIC);
                
                cerr << "Original translation: " << T << endl;
                cerr << "Reprojection error reported by camera: " << rms << endl;
                
                // convert to rotation vector and then remove 90 degree offset
                Rodrigues(R, rvec);
                rvec.at<double>(1,0) -= 1.570796327;
                
                // equal rotation applied to each image...not necessarily needed
                rvec = rvec/2;
                Rodrigues(rvec, rmat1);
                invert(rmat1,rmat2);
                
                initUndistortRectifyMap(cameraMatrix1, distCoeffs1, rmat1,
                                        getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, imageSize, 1,imageSize, 0), imageSize, CV_32FC1, mapX1, mapY1);
                initUndistortRectifyMap(cameraMatrix2, distCoeffs2, rmat2,
                                        getOptimalNewCameraMatrix(cameraMatrix2, distCoeffs2, imageSize, 1, imageSize, 0), imageSize, CV_32FC1, mapX2, mapY2);
                
                
                // reproject points in camera 1 since its rotation has been changed
                // need to find the translation between cameras based on the new camera 1 orientation
                for  (int i = 0; i < imagePoints1.size(); i++)
                {
                    Mat pointsMat1 = Mat(imagePoints1[i]);
                    Mat pointsMat2 = Mat(imagePoints2[i]);
                    
                    
                    undistortPoints(pointsMat1, imagePoints1[i], cameraMatrix1, distCoeffs1, rmat1,getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, imageSize, 1, imageSize, 0));
                    undistortPoints(pointsMat2, imagePoints2[i], cameraMatrix2, distCoeffs2, rmat2,getOptimalNewCameraMatrix(cameraMatrix2, distCoeffs2, imageSize, 1, imageSize, 0));
                    
                    pointsMat1.release();
                    pointsMat2.release();
                }
                
                Mat temp1, temp2;
                R.release();
                T.release();
                E.release();
                F.release();
                
                // TODO: remove this
                // CalcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0]);
                // objectPoints.resize(imagePoints1.size(),objectPoints[0]);
                
                stereoCalibrate(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, E, F,
                                TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 1000, 0.01),
                                CV_CALIB_FIX_INTRINSIC);
                
                // need to alter translation matrix so
                // [0] = distance in X direction (right from perspective of camera 1 is positive)
                // [1] = distance in Y direction (away from camera 1 is positive)
                // [2] = distance in Z direction (up is positive)
                translation = T;
                double temp = -translation.at<double>(0,0);
                translation.at<double>(0,0) = translation.at<double>(2,0);
                translation.at<double>(2,0) = temp;
                
                cerr << "Translation reproj: " << translation << endl;
                Rodrigues(R, rvec);
                cerr << "Reprojected rvec: " << rvec << endl;
                
                imagePoints1.clear();
                imagePoints2.clear();
                
                rvec.release();
                rmat1.release();
                rmat2.release();
                R.release();
                T.release();
                E.release();
                F.release();
                
                rotationCalibrated = true;
            }
        }
        imshow("Image View1", image1);
        imshow("Image View2", image2);
    }
}

