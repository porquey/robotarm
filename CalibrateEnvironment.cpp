#include "CalibrateEnvironment.h"


void CalibrateEnvironment(VideoCapture& inputCapture1, VideoCapture& inputCapture2)
{
  Size boardSize;
  boardSize.width = BOARD_WIDTH;
  boardSize.height = BOARD_HEIGHT;

  const string fileName1 = "CameraIntrinsics1.xml";
  const string fileName2 = "CameraIntrinsics2.xml";

  cout << "Attempting to open configuration files" << endl;
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
    printf("Could not load camera intrinsics\n");
  }
  printf("Loaded intrinsics\n");

  Mat translation;
  Mat image1, image2;
  Mat mapX1, mapX2, mapY1, mapY2;
  inputCapture1.read(image1);
  Size imageSize = image1.size();
  bool rotationCalibrated = false;
  bool translationCalibrated = false;
  bool found1 = false;
  bool found2 = false;


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
        printf("Cancelling...\n");
        return;
    }
      else if(c == 's' && translationCalibrated)
    {
      printf("Saving...\n");
      const string fileName = "EnvironmentCalibration.xml";
      FileStorage fs(fileName, FileStorage::WRITE);
      fs << "Camera_Matrix_1" << cameraMatrix1;
      fs << "Camera_Matrix_2" << cameraMatrix2;
      fs << "Mapping_X_1" << mapX1;
      fs << "Mapping_Y_1" << mapY1;
      fs << "Mapping_X_2" << mapX2;
      fs << "Mapping_Y_2" << mapY2;
      fs << "Translation" << translation;
        return;
    }
    else if (c == 't' && rotationCalibrated)
    {
      printf("Finding translation\n");
      Mat rvecs, tvecs;
      vector<Point2f> imagePoints1, imagePoints2;
      vector<Point3f> objectPoints1, objectPoints2;

      printf("Finding corners...\n");

      found1 = findChessboardCorners(image1, boardSize, imagePoints1,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
          CV_CALIB_CB_NORMALIZE_IMAGE);

      found2 = findChessboardCorners(image2, boardSize, imagePoints2,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
          CV_CALIB_CB_NORMALIZE_IMAGE);

      printf("Found corners.\n");


      if (found1 & found2)
      {
        for( int i = 0; i < BOARD_HEIGHT; ++i )
            for( int j = 0; j < BOARD_WIDTH; ++j )
                objectPoints1.push_back(Point3f(float( j*(float)SQUARE_SIZE_SMALL ), float( i*(float)SQUARE_SIZE_SMALL ), 0));

        for( int i = 0; i < BOARD_HEIGHT; ++i )
            for( int j = 0; j < BOARD_WIDTH; ++j )
                objectPoints2.push_back(Point3f(float( j*(float)SQUARE_SIZE_SMALL ), float( i*(float)SQUARE_SIZE_SMALL ), 0));

        Mat rvecs1, tvecs1, rvecs2, tvecs2, tdiff;
        Mat tvecsT;
        Mat mult = (Mat_<double>(3,1) << -1, 1, 1);
        printf("Solving PnP...\n");


        solvePnP(objectPoints1, imagePoints1, cameraMatrix1, distCoeffs1, rvecs1, tvecs1, 0);
        solvePnP(objectPoints2, imagePoints2, cameraMatrix2, distCoeffs2, rvecs2, tvecs2, 0);

        printf("Solved PnP.\n");

        cout<< "translation 2: before " << tvecs2 << endl;


        //transpose(tvecs2, tvecsT);
        flip(tvecs2, tvecs2, 0);

        cout<< "translation 2: after flip " << tvecs2 << endl;

        multiply(tvecs2, mult, tvecs2, 1);

        cout<< "translation 2: after mult " << tvecs2 << endl;


        translation = tvecs1 - tvecs2;

        cout<< "translation 1: " << tvecs1 << endl;
        cout<< "translation 2: " << tvecs2 << endl;
        //cout<< "translation 2 transpose: " << tvecsT << endl;


        cout<< "translation diff: " << translation << endl;

        rvecs1.release();
        tvecs1.release();
        rvecs2.release();
        tvecs2.release();

        translationCalibrated = true;
        printf("Found distance...\n");
      }
    }
    else if (c == 'r')
    {
      printf("Finding rotation\n");

      Mat rvecs, tvecs;
      vector<Point2f> imagePoints1, imagePoints2;
      vector<Point3f> objectPoints1, objectPoints2;

      printf("Finding corners...\n");

      found1 = findChessboardCorners(image1, boardSize, imagePoints1,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
          CV_CALIB_CB_NORMALIZE_IMAGE);

      found2 = findChessboardCorners(image2, boardSize, imagePoints2,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
          CV_CALIB_CB_NORMALIZE_IMAGE);

      printf("Found corners.\n");


      if (found1 & found2)
      {
        for( int i = 0; i < BOARD_HEIGHT; ++i )
            for( int j = 0; j < BOARD_WIDTH; ++j )
                objectPoints1.push_back(Point3f(float( j*(float)SQUARE_SIZE ), float( i*(float)SQUARE_SIZE ), 0));

        for( int i = 0; i < BOARD_HEIGHT; ++i )
            for( int j = 0; j < BOARD_WIDTH; ++j )
                objectPoints2.push_back(Point3f(float( j*(float)SQUARE_SIZE ), float( i*(float)SQUARE_SIZE ), 0));

        Mat rvecs1, tvecs1, rvecs2, tvecs2, tdiff, rmat1, rmat2;

        printf("Solving PnP...\n");


        solvePnP(objectPoints1, imagePoints1, cameraMatrix1, distCoeffs1, rvecs1, tvecs1, 0);
        solvePnP(objectPoints2, imagePoints2, cameraMatrix2, distCoeffs2, rvecs2, tvecs2, 0);

        printf("Solved PnP.\n");


        cout << "rvecs1: " << rvecs1 << endl;
        cout << "tvecs1: " << tvecs1 << endl;
        cout << "rvecs2: " << rvecs2 << endl;
        cout << "tvecs2: " << tvecs2 << endl;

        Rodrigues(rvecs1, rmat1);
        Rodrigues(rvecs2, rmat2);

        invert(rmat1, rmat1);
        invert(rmat2, rmat2);


        initUndistortRectifyMap(cameraMatrix1, distCoeffs1, rmat1,
        getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, imageSize, 1, imageSize, 0),
        imageSize, CV_32FC1, mapX1, mapY1);

        initUndistortRectifyMap(cameraMatrix2, distCoeffs2, rmat2,
        getOptimalNewCameraMatrix(cameraMatrix2, distCoeffs2, imageSize, 1, imageSize, 0),
        imageSize, CV_32FC1, mapX2, mapY2);


        rvecs1.release();
        tvecs1.release();
        rvecs2.release();
        tvecs2.release();

        rotationCalibrated = true;
      }
    }
    imshow("Camera1", image1);
    imshow("Camera2", image2);

  }
}
