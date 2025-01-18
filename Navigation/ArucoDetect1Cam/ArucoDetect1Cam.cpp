// Import the aruco module in OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <unistd.h>
using namespace cv;
using namespace std;

std::vector<cv::Point2f> centerCordinatesInPixels(const std::vector<std::vector<cv::Point2f>> &corners)
{
	std::vector<cv::Point2f> centers(corners.size());
	for(std::size_t i = 0; i < corners.size(); i++)
	{
		centers.at(i) = (corners.at(i).at(0) + corners.at(i).at(2))/2;
	}
	return centers;
}

int main(){
//	cv::Mat markerImage;
//	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
//	for(int i = 0; i<15; i++)
//	{
//		cv::aruco::generateImageMarker(dictionary, i, 200, markerImage, 1);
//		cv::imwrite("marker"+to_string(i)+".png", markerImage);
//	}
			cv::VideoCapture inputVideo1,inputVideo2;
			inputVideo1.open(4);
			cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
			cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
			cv::aruco::ArucoDetector detector(dictionary, detectorParams);

	//	Camera matrix and distortion co-efficients from camera calibration
			cv::Matx33f cameraMatrix1(3.1423828914945648e+03, 0., 9.5950000000000000e+02, 0.,
					   3.1423828914945648e+03, 5.3950000000000000e+02, 0., 0., 1. );
			cv::Vec<float , 5> distCoeffs1( -6.3475904043817322e-01, 1.6701484899182653e+01, 0., 0.,
					   -1.5245763613467119e+02 );
			float markerLength = 0.05;

	//	 Set coordinate system
			cv::Mat objPoints(4, 1, CV_32FC3);
			objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
			objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
			objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
			objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

			while (inputVideo1.grab())
			{
				cv::Mat image1,imageCopy1;

				inputVideo1.retrieve(image1);

				image1.copyTo(imageCopy1);

				std::vector<int> ids1;	// ids of detected markers in both cameras

				std::vector<std::vector<cv::Point2f>> corners1, rejected1;

				detector.detectMarkers(image1, corners1, ids1, rejected1);

				std::vector<cv::Point2f> centers1(corners1.size());
	//	 if at least one marker detected
				if (ids1.size() > 0)
					{
						cv::aruco::drawDetectedMarkers(imageCopy1, corners1, ids1);

						int nMarkers1 = corners1.size();

						std::vector<cv::Vec3d> rvecs1(nMarkers1), tvecs1(nMarkers1);

						// Calculating pose for each marker
						for (int i = 0; i < nMarkers1; i++) {
							solvePnP(objPoints, corners1.at(i), cameraMatrix1, distCoeffs1, rvecs1.at(i), tvecs1.at(i));
						}

						// Drawing axis for each marker
						for(unsigned int i = 0; i < ids1.size(); i++) {
							cv::drawFrameAxes(imageCopy1, cameraMatrix1, distCoeffs1, rvecs1[i], tvecs1[i], 0.1);
						}

						centers1 = centerCordinatesInPixels(corners1);

						for(int i = 0; i<nMarkers1; i++)
						{
							cout<< ids1.at(i) << "\t"<< "\t" << centers1.at(i) << "\t"<< std::endl;
						}
//						sleep(20);
						cout<< std::endl;
						cout<< std::endl;
						cout<< std::endl;
					}

		cv::imshow("out1", imageCopy1);
		char key = (char)
		cv::waitKey(20);
		if (key == 27)
		    break;
	}

}