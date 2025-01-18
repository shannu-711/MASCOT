#include <iostream>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {

// Creating Charuco Board
   int squaresX = 3;
   int squaresY = 5;
   int squareLength = 400;
   int markerLength = 250;
   int margins = squareLength - markerLength;

   aruco::Dictionary dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

   Size imageSize;
   imageSize.width = squaresX * squareLength + 2 * margins;
   imageSize.height = squaresY * squareLength + 2 * margins;

   aruco::CharucoBoard board(Size(squaresX, squaresY), (float)squareLength, (float)markerLength, dictionary);

   // show created board
   Mat boardImage;
   board.generateImage(imageSize, boardImage, margins, 1);

       imshow("board", boardImage);
       waitKey(0);

   imwrite("charucoBoard.png", boardImage);


		// int squaresX = 3;
	    // int squaresY = 5;
	    // float squareLength = 0.048;		// In meters
	    // float markerLength = 0.036;		// In meters
	    // string outputFile = "cameraMatrices10.txt";

	    // bool showChessboardCorners = true;

	    // int calibrationFlags = 0;
	    // float aspectRatio = 1;
	    //     calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
	    //     calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
	    //     calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;

	    // aruco::DetectorParameters detectorParams = aruco::DetectorParameters();

	    // VideoCapture inputVideo;
	    // int waitTime;
	    //     inputVideo.open(10);
	    //     waitTime = 10;

	    // aruco::Dictionary dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	    // // Create charuco board object
	    // aruco::CharucoBoard board(Size(squaresX, squaresY), squareLength, markerLength, dictionary);
	    // aruco::CharucoParameters charucoParams;

	    //     charucoParams.tryRefineMarkers = true;

	    // aruco::CharucoDetector detector(board, charucoParams, detectorParams);

	    // // Collect data from each frame
	    // vector<Mat> allCharucoCorners;
	    // vector<Mat> allCharucoIds;

	    // vector<vector<Point2f>> allImagePoints;
	    // vector<vector<Point3f>> allObjectPoints;

	    // vector<Mat> allImages;
	    // Size imageSize;

	    // while(inputVideo.grab()) {
	    //     Mat image, imageCopy;
	    //     inputVideo.retrieve(image);

	    //     vector<int> markerIds;
	    //     vector<vector<Point2f>> markerCorners, rejectedMarkers;
	    //     Mat currentCharucoCorners;
	    //     Mat currentCharucoIds;
	    //     vector<Point3f> currentObjectPoints;
	    //     vector<Point2f> currentImagePoints;

	    //     // Detect ChArUco board
	    //     detector.detectBoard(image, currentCharucoCorners, currentCharucoIds);

	    //     // Draw results
	    //     image.copyTo(imageCopy);
	    //     if(!markerIds.empty()) {
	    //         aruco::drawDetectedMarkers(imageCopy, markerCorners);
	    //     }

	    //     if(currentCharucoCorners.total() > 3) {
	    //         aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
	    //     }

	    //     putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
	    //             Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

	    //     imshow("out", imageCopy);

	    //     // Wait for key pressed
	    //     char key = (char)waitKey(waitTime);

	    //     if(key == 27) {
	    //         break;
	    //     }

	    //     if(key == 'c' && currentCharucoCorners.total() > 3) {
	    //         // Match image points
	    //         board.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

	    //         if(currentImagePoints.empty() || currentObjectPoints.empty()) {
	    //             cout << "Point matching failed, try again." << endl;
	    //             continue;
	    //         }

	    //         cout << "Frame captured" << endl;

	    //         allCharucoCorners.push_back(currentCharucoCorners);
	    //         allCharucoIds.push_back(currentCharucoIds);
	    //         allImagePoints.push_back(currentImagePoints);
	    //         allObjectPoints.push_back(currentObjectPoints);
	    //         allImages.push_back(image);

	    //         imageSize = image.size();
	    //     }
	    // }

	    // if(allCharucoCorners.size() < 4) {
	    //     cerr << "Not enough corners for calibration" << endl;
	    //     return 0;
	    // }

	    // Mat cameraMatrix, distCoeffs;

	    // if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
	    //     cameraMatrix = Mat::eye(3, 3, CV_64F);
	    //     cameraMatrix.at<double>(0, 0) = aspectRatio;
	    // }

	    // // Calibrate camera using ChArUco
	    // double repError = calibrateCamera(
	    //     allObjectPoints, allImagePoints, imageSize,
	    //     cameraMatrix, distCoeffs, noArray(), noArray(), noArray(),
	    //     noArray(), noArray(), calibrationFlags
	    // );

	    // bool saveOk =  saveCameraParams(
	    //     outputFile, imageSize, aspectRatio, calibrationFlags,
	    //     cameraMatrix, distCoeffs, repError
	    // );

	    // if(!saveOk) {
	    //     cerr << "Cannot save output file" << endl;
	    //     return 0;
	    // }

	    // cout << "Rep Error: " << repError << endl;
	    // cout << "Calibration saved to " << outputFile << endl;

	    // // Show interpolated charuco corners for debugging
	    // if(showChessboardCorners) {
	    //     for(size_t frame = 0; frame < allImages.size(); frame++) {
	    //         Mat imageCopy = allImages[frame].clone();

	    //         if(allCharucoCorners[frame].total() > 0) {
	    //             aruco::drawDetectedCornersCharuco(
	    //                 imageCopy, allCharucoCorners[frame], allCharucoIds[frame]
	    //             );
	    //         }

	    //         imshow("out", imageCopy);
	    //         char key = (char)waitKey(0);
	    //         if(key == 27) {
	    //             break;
	    //         }
	    //     }
	    // }

    return 0;
}