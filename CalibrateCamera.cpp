#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define CAPTURE_DELAY 500
#define SQUARE_SIZE 25.2
#define BOARD_WIDTH 9
#define BOARD_HEIGHT 6

using namespace cv;
using namespace std;

enum calibration_state_t {UNCALIBRATED, CALIBRATING, CALIBRATED};

class BoardSettings{
public:
  int cornerNum;
  int squareSize;
  Size boardSize;
};

Mat distCoeffs;
Mat cameraMatrix;

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.clear();

    for( int i = 0; i < boardSize.height; ++i )
        for( int j = 0; j < boardSize.width; ++j )
            corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
}

static void saveCalibration(const string fileName, Mat& cameraMatrix,
                            Mat& distCoeffs, Mat& mapX, Mat& mapY){
  FileStorage fs(fileName, FileStorage::WRITE);
  fs << "Camera_Matrix" << cameraMatrix;
  fs << "Distortion_Coefficients" << distCoeffs;
  fs << "Mapping_X" << mapX;
  fs << "Mapping_Y" << mapY;
}

static bool runCalibration( BoardSettings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0]);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs,CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
    return ok;
}


int main(int argc, char* argv[]){

  int imageNum;
  int cameraID;

  calibration_state_t state = UNCALIBRATED;


  if(argc != 3){
    printf("Wrong number of parameters");
    return -1;
  }

  BoardSettings s;

  //Get input parameters
  imageNum = atoi(argv[1]);
  cameraID = atoi(argv[2]);
  s.boardSize.width = BOARD_WIDTH;
  s.boardSize.height = BOARD_HEIGHT;
  s.cornerNum = s.boardSize.width * s.boardSize.height;
  s.squareSize = (float)SQUARE_SIZE;

  //Check input parameters
  if (s.boardSize.width < 4){
    printf("Invalid board width");
    return -1;
  }
  if (s.boardSize.height < 2){
    printf("Invalid board height");
    return -1;
  }
  if (imageNum < 1){
    printf("Invalid board width");
    return -1;
  }
  if (cameraID < 0){
    printf("Invalid camera ID");
    return -1;
  }

  printf("Board Width: %d\n", s.boardSize.width);
  printf("Board Height: %d\n", s.boardSize.height);
  printf("Num of Iterations: %d\n", imageNum);
  printf("Camera ID: %d\n", cameraID);

  VideoCapture inputCapture(cameraID);
  inputCapture.set(CV_CAP_PROP_FRAME_WIDTH,640);
  inputCapture.set(CV_CAP_PROP_FRAME_HEIGHT,480);

  //Get files names appended with camera ID
  stringstream nameStream;
  nameStream << "CameraIntrinsics" << cameraID << ".xml";

  const string fileName = nameStream.str();

  cout << "Attempting to open configuration file" << endl;
  FileStorage fs(fileName, FileStorage::READ);
  if (!fs.isOpened()){
    cout << "Could not open the configuration file: \"" << fileName << "\""  << endl;
  }
  cout << "Opened the configuration file: \"" << fileName << "\""  << endl;

  //Attemp to read in current calibrated values
  fs["Camera_Matrix"] >> cameraMatrix;
  fs["Distortion_Coefficients"] >> distCoeffs;
  Mat image;
  Mat mapX, mapY;

  if (!inputCapture.isOpened()){
    printf("Could not open camera");
    return -1;
  }

  inputCapture.read(image);
  Size imageSize = image.size();


  if (cameraMatrix.data != NULL && distCoeffs.data != NULL){
    printf("Loaded intrinsics\n");
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
    imageSize, CV_32FC1, mapX, mapY);
    //Change state
    state = CALIBRATED;
  }
  else{
    printf("Could not find intrinsics\n");
  }

  vector<vector<Point2f> > imagePoints;
  vector<Point2f> pointBuffer;
  bool found;
  bool showUndistorted = false;
  clock_t prevTimeStamp = 0;
  int iterations;

  while(inputCapture.isOpened()){
    inputCapture.read(image);

    //Undistort image if it has been calibrated
    if (state == CALIBRATED){
      if (!showUndistorted){
        Mat t = image.clone();
        remap(t, image, mapX, mapY, INTER_LINEAR);
        t.release();
      }

      char c = waitKey(15);
      if (c == 'g'){
        printf("Beginning calibration...\n");
        state = CALIBRATING;
        iterations = 0;
      }
      else if(c == 't'){
        showUndistorted = !showUndistorted;
      }
    }

    if (state == UNCALIBRATED){
      char c = waitKey(15);
      if (c == 'g'){
        printf("Beginning calibration...\n");
        state = CALIBRATING;
        iterations = 0;
      }
    }

    if (state == CALIBRATING){
      if (iterations == imageNum){
        vector<Mat> rvecs, tvecs;
        vector<float> reprojErrs;
        double totalAvgErr = 0;

        bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs);

        cout << (ok ? "Calibration succeeded" : "Calibration failed") << endl;

        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_32FC1, mapX, mapY);

        saveCalibration(fileName, cameraMatrix, distCoeffs, mapX, mapY);
        cout << "Saved calibration settings" << endl;
        //Change state
        state = CALIBRATED;
      }

      char c = waitKey(15);
      if (c == 's'){
        printf("Stopped calibration...\n");
        state = CALIBRATED;
        iterations = 0;
      }
      else if(c == 'c'){
        //Try find chessboard corners
        //ADAPTIVE_THRESH -> use adaptive thresholding to convert image to B&W
        //FAST_CHECK -> Terminates call earlier if no chessboard in image
        //NORMALIZE_IMAGE -> normalize image gamma before thresholding
        //FILTER_QUADS -> uses additional criteria to filter out false quads
        found = findChessboardCorners(image, s.boardSize, pointBuffer,
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
            CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);

        if (found && (pointBuffer.size() >= s.cornerNum)){
          //If time delay passed refine accuracy and store
          if ((clock() - prevTimeStamp) > CAPTURE_DELAY * 1e-3*CLOCKS_PER_SEC){
            Mat imageGray;
            cvtColor(image, imageGray, COLOR_BGR2GRAY);
            //Refines corner locations
            //Size(11,11) -> size of the search window
            //Size(-1,-1) -> indicates no dead zone in search size
            //TermCriteria -> max 30 iteration, to get acuraccy of 0.1
            cornerSubPix(imageGray, pointBuffer, Size(11,11), Size(-1, -1),
            TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

            drawChessboardCorners(image, s.boardSize, Mat(pointBuffer), found);
            imshow("Image View", image);

            //User verifies the correct corners have been found
            c = waitKey(0);
            if (c == 'y'){
              //Store the points and store time stamp
              imagePoints.push_back(pointBuffer);
              prevTimeStamp = clock();
              iterations++;
              cout << "Iteration: " << iterations << endl;
            }
          }
        }
      }
    }
    imshow("Image View", image);
  }
}
