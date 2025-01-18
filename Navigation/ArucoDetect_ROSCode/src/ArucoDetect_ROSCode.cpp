// Reference Marker Index = 0
// Target sapcecraft Index = 7
// Chaser spacecraft Index = 1


// Import the aruco module in OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cmath>
#include <string>       // for std::string

#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <sstream>

using namespace cv;
using namespace std;

// Calculates yaw angle from rotation vector
std::vector<float> zAngle(const std::vector<cv::Vec3d> &rvecs)
{
	std::vector<float> zAngle(rvecs.size());
	std::vector<cv::Mat> rMats(rvecs.size());
	for(std::size_t i = 0; i<rvecs.size(); i++)
	{
		cv::Rodrigues(rvecs.at(i),rMats.at(i));	// converting rvec to rotation matrix
//	Calculating yaw angle from rotation matrix by 3-2-1  Euler angle sequence
		    zAngle.at(i) = -atan2(rMats.at(i).at<double>(0,1), rMats.at(i).at<double>(0,0))*180/M_PI;
	}
	return zAngle;
}
//	Calculating center co-ordinates from corner co-ordinates
std::vector<cv::Point2f> centerCordinatesInPixels(const std::vector<std::vector<cv::Point2f>> &corners)
{
	std::vector<cv::Point2f> centers(corners.size());
	for(std::size_t i = 0; i < corners.size(); i++)
	{
		centers.at(i) = (corners.at(i).at(0) + corners.at(i).at(2))/2;
	}
	return centers;
}

//	Converting center co-ordinates in pixels to cms for camera-1
std::vector<cv::Point2f> centerCordinatesInCms_Cam1(const std::vector<cv::Point2f> &centersInPixels)
{
	std::vector<cv::Point2f> centersInCms(centersInPixels.size());

	float height = 235; // height of lens in cms

//	Calibration constants
	float calibrationConstant1x = height*(0.178/242);
	float calibrationConstant1y = height*(0.17924/242);
	for(std::size_t i = 0; i < centersInPixels.size(); i++)
	{
		centersInCms.at(i).x = (centersInPixels.at(i).x) * calibrationConstant1x;
		centersInCms.at(i).y = (centersInPixels.at(i).y) * calibrationConstant1y;
	}
	return centersInCms;
}

//	Converting center co-ordinates in pixels to cms for camera-2
std::vector<cv::Point2f> centerCordinatesInCms_Cam2(const std::vector<cv::Point2f> &centersInPixels)
{
	std::vector<cv::Point2f> centersInCms(centersInPixels.size());

	float height = 235; // height of lens in cms

//	Calibration constants
	float calibrationConstant2x = height*(0.18137/242);
	float calibrationConstant2y = height*(0.17955/242);
	for(std::size_t i = 0; i < centersInPixels.size(); i++)
	{
		centersInCms.at(i).x = (centersInPixels.at(i).x) * calibrationConstant2x;
		centersInCms.at(i).y = (centersInPixels.at(i).y) * calibrationConstant2y;
	}
	return centersInCms;
}

//	Converting center co-ordinates from camera frame to inertial frame
cv::Point2f Cam2InertialCordinates(cv::Point2f &RefCordinates, cv::Point2f &center, cv::Mat &RMat)
{
	cv::Point2f InertialCordinates = center - RefCordinates;

	cv::Mat_<double> temp(3,1);
//	Converting 2D point to 3x1 matrix to make matrix multiplication feasible
	    temp(0,0)=InertialCordinates.x;
	    temp(1,0)=InertialCordinates.y;
	    temp(2,0)=0.0;
// Converting to inertial frame by multiplying rotation matrix from camera frame to inertial frame
	    cv::Mat_<double> Temp = RMat*temp;
	    InertialCordinates.x = Temp(0,0);
	    InertialCordinates.y = Temp(1,0);
	return InertialCordinates;
}

//	Fusing data from both cameras
std::vector<cv::Point2f> FinalCordinates(std::vector<int> &ids1, std::vector<int> &ids2, std::vector<cv::Point2f> &InertialCordinates1, std::vector<cv::Point2f> &InertialCordinates2)
{
	cv::Point2f temp(0);
	float a1=.5,a2=.5;	// weights of both cameras
	std::vector<cv::Point2f> finalCordinates;
	finalCordinates.clear();

// 	For target space-craft(it has id fixed, 7)
	int id1 = -1,id2 = -1;
		for(std::size_t i = 0; i<ids1.size(); i++)
		{
			if(ids1.at(i) == 7)
			{
				id1 = i;
				break;
			}
		}
		for(std::size_t i = 0; i<ids2.size(); i++)
		{
			if(ids2.at(i) == 7)
			{
				id2 = i;
				break;
			}
		}
		if(id1 == -1 && id2 == -1)	// if not detected by both cameras
		{
			std::cout<< "ERROR!!!"<< std::endl<<"One spacecraft is out of arena!"<< std::endl;
		}
		else if(id1 != -1 && id2 != -1)	// if detected by both cameras
		{
			temp = a1*InertialCordinates1.at(id1) + a2*InertialCordinates2.at(id2);
			finalCordinates.push_back(temp);
		}
//		if detected by only one camera
		else if(id1 != -1){
			temp = InertialCordinates1.at(id1);
			finalCordinates.push_back(temp);}
		else{
			temp = InertialCordinates2.at(id2);
			finalCordinates.push_back(temp);}

// 	For chaser space-craft(it has id fixed, 1)
	int id11 = -1,id22 = -1;
		for(std::size_t i = 0; i<ids1.size(); i++)
		{
			if(ids1.at(i) == 1)
			{
				id11 = i;
				break;
			}
		}
		for(std::size_t i = 0; i<ids2.size(); i++)
		{
			if(ids2.at(i) == 1)
			{
				id22 = i;
				break;
			}
		}
		if(id11 == -1 && id22 == -1)
		{
			std::cout<< "ERROR!!!"<< std::endl<<"One spacecraft is out of arena!"<< std::endl;
		}
		else if(id11 != -1 && id22 != -1){
			temp = a1*InertialCordinates1.at(id11) + a2*InertialCordinates2.at(id22);
			finalCordinates.push_back(temp);}
		else if(id11 != -1){
			temp = InertialCordinates1.at(id11);
			finalCordinates.push_back(temp);}
		else{
			temp = InertialCordinates2.at(id22);
			finalCordinates.push_back(temp);}

	return finalCordinates;
}

std::vector<float> FinalZAngle(std::vector<int> &ids1, std::vector<int> &ids2, std::vector<float> &zAngle1, std::vector<float> &zAngle2)
{
	float temp=0;
	std::vector<float> zAngleVec(0);
	float a1=.5,a2=.5;
	int id1 = -1,id2 = -1;
	for(std::size_t i = 0; i<ids1.size(); i++)
	{
		if(ids1.at(i) == 7)
		{
			id1 = i;
			break;
		}
	}
	for(std::size_t i = 0; i<ids2.size(); i++)
	{
		if(ids2.at(i) == 7)
		{
			id2 = i;
			break;
		}
	}
	if(id1 == -1 && id2 == -1)
	{
		std::cout<< "ERROR!!!"<< std::endl<<"One spacecraft is out of arena!"<< std::endl;
	}
	else if(id1 != -1 && id2 != -1){
		temp = a1*zAngle1.at(id1) + a2*zAngle2.at(id2);
		temp = temp - ceil(temp/360-0.5)*360;			// This is for mapping angles to -180 - 180 degrees
		zAngleVec.push_back(temp);}
	else if(id1 != -1){
		temp = zAngle1.at(id1);
		temp = temp - ceil(temp/360-0.5)*360;			// This is for mapping angles to -180 - 180 degrees
		zAngleVec.push_back(temp);}
	else{
		temp = zAngle2.at(id2);
		temp = temp - ceil(temp/360-0.5)*360;			// This is for mapping angles to -180 - 180 degrees
		zAngleVec.push_back(temp);}


	int id11 = -1,id22 = -1;
	for(std::size_t i = 0; i<ids1.size(); i++)
	{
		if(ids1.at(i) == 1)
		{
			id11 = i;
			break;
		}
	}
	for(std::size_t i = 0; i<ids2.size(); i++)
	{
		if(ids2.at(i) == 1)
		{
			id22 = i;
			break;
		}
	}
	if(id11 == -1 && id22 == -1)
	{
		std::cout<< "ERROR!!!"<< std::endl<<"One spacecraft is out of arena!"<< std::endl;
	}
	else if(id11 != -1 && id22 != -1){
		temp = a1*zAngle1.at(id11) + a2*zAngle2.at(id22);
		temp = temp - ceil(temp/360-0.5)*360;			// This is for mapping angles to -180 - 180 degrees
		zAngleVec.push_back(temp);}
	else if(id11 != -1){
		temp = zAngle1.at(id11);
		temp = temp - ceil(temp/360-0.5)*360;			// This is for mapping angles to -180 - 180 degrees
		zAngleVec.push_back(temp);}
	else{
		temp = zAngle2.at(id22);
		temp = temp - ceil(temp/360-0.5)*360;			// This is for mapping angles to -180 - 180 degrees
		zAngleVec.push_back(temp);}

	return zAngleVec;
}

int main(int argc, char **argv){

// 	Creating aruco marker

//	cv::Mat markerImage;
//	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
//	cv::aruco::generateImageMarker(dictionary, 23, 200, markerImage, 1);
//	cv::imwrite("marker23.png", markerImage);

// 	Initialisng ROS and publishing target and chaser co-ordinates
	ros::init(argc, argv, "Position");
	ros::NodeHandle n;

	ros::Publisher pub1 = n.advertise<geometry_msgs::Vector3>("TargetPosition",1);
	ros::Publisher pub2 = n.advertise<geometry_msgs::Vector3>("ChaserPosition",1);
	ros::Rate rate(10);

	while(ros::ok())
	{
		geometry_msgs::Vector3 pose1,pose2;

// 	Capturing frames from camera and detecting aruco markers
		cv::VideoCapture inputVideo1,inputVideo2;
		inputVideo1.open(10);
		inputVideo2.open(4);

		cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
		cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
		cv::aruco::ArucoDetector detector(dictionary, detectorParams);

//	Camera matrix and distortion co-efficients from camera calibration
		cv::Matx33f cameraMatrix1(3.1423828914945648e+03, 0., 9.5950000000000000e+02, 0.,
				   3.1423828914945648e+03, 5.3950000000000000e+02, 0., 0., 1. );
		cv::Matx33f cameraMatrix2(1.5661071096989331e+03, 0., 9.5950000000000000e+02, 0.,
				   1.5661071096989331e+03, 5.3950000000000000e+02, 0., 0., 1.);
		cv::Vec<float , 5> distCoeffs1( -6.3475904043817322e-01, 1.6701484899182653e+01, 0., 0.,
				   -1.5245763613467119e+02 );
		cv::Vec<float , 5> distCoeffs2( 2.1155464301672353e-01, -1.3306745479489419e+00, 0., 0.,
				   1.8318015956259908e+00);
		float markerLength = 0.05;

		bool flag = true;	// flag to run Initialization only once

		cv::Mat RotMat12N, RotMat22N;	// Rotation matrices to convert from camera frame to inertial frame
		cv::Point2f RefCenter1, RefCenter2;	// Center of reference marker in both cameras

//	 Set coordinate system
		cv::Mat objPoints(4, 1, CV_32FC3);
		objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

		while (inputVideo1.grab() && inputVideo2.grab())
		{
			cv::Mat image1, image2, imageCopy1, imageCopy2;

			inputVideo1.retrieve(image1);
			inputVideo2.retrieve(image2);

			image1.copyTo(imageCopy1);
			image2.copyTo(imageCopy2);

			std::vector<int> ids1 , ids2;	// ids of detected markers in both cameras

			std::vector<std::vector<cv::Point2f>> corners1, rejected1;	// vector of vector of corners of detected valid aruco markers and rejected shapes
			std::vector<std::vector<cv::Point2f>> corners2, rejected2;

			detector.detectMarkers(image1, corners1, ids1, rejected1);
			detector.detectMarkers(image2, corners2, ids2, rejected2);

			std::vector<cv::Point2f> centers1(corners1.size()), centersInCms1(corners1.size());	// vector of centers of valid detected markers in pixels and in cms
			std::vector<cv::Point2f> centers2(corners2.size()), centersInCms2(corners2.size());

			std::vector<float> zAngle1(corners1.size()), zAngle2(corners2.size());	// yaw angle w.r.t. camera frame of all detected markers

//	 if at least one marker detected
			if (ids1.size() > 0 || ids2.size() > 0)
				{
					cv::aruco::drawDetectedMarkers(imageCopy1, corners1, ids1);
					cv::aruco::drawDetectedMarkers(imageCopy2, corners2, ids2);

					int nMarkers1 = corners1.size();	// number of markers
					int nMarkers2 = corners2.size();

					std::vector<cv::Vec3d> rvecs1(nMarkers1), tvecs1(nMarkers1);	//rotation and translation vectors for each marker
					std::vector<cv::Vec3d> rvecs2(nMarkers2), tvecs2(nMarkers2);

					// Calculating pose for each marker
					for (int i = 0; i < nMarkers1; i++) {
						solvePnP(objPoints, corners1.at(i), cameraMatrix1, distCoeffs1, rvecs1.at(i), tvecs1.at(i));
					}
					for (int i = 0; i < nMarkers2; i++)	{
						solvePnP(objPoints, corners2.at(i), cameraMatrix2, distCoeffs2, rvecs2.at(i), tvecs2.at(i));
					}

					// Drawing axis for each marker
					for(unsigned int i = 0; i < ids1.size(); i++) {
						cv::drawFrameAxes(imageCopy1, cameraMatrix1, distCoeffs1, rvecs1[i], tvecs1[i], 0.1);
					}
					for(unsigned int i = 0; i < ids2.size(); i++) {
						cv::drawFrameAxes(imageCopy2, cameraMatrix2, distCoeffs2, rvecs2[i], tvecs2[i], 0.1);
					}

//		calculating centers of each marker
					centers1 = centerCordinatesInPixels(corners1);
					centers2 = centerCordinatesInPixels(corners2);
//		converting centers in pixels to centers in cms
					centersInCms1 = centerCordinatesInCms_Cam1(centers1);
					centersInCms2 = centerCordinatesInCms_Cam2(centers2);
//		calculating the yaw-angle from rvecs
					zAngle1 = zAngle(rvecs1);
					zAngle2 = zAngle(rvecs2);

					int ReferenceId = 0;	// fixing the reference marker id
					int ReferenceIdIndex1 = -1, ReferenceIdIndex2 = -1;	// Index of reference marker in the vector of ids
					float RefMarkerAngle1, RefMarkerAngle2;

//		Initializing and calculating the cordinates of reference tag and Camera frame to Inertial frame Rotation Matrix
					for(int id = 0; id < nMarkers1 ; id++)
					{
						if(ids1.at(id) == ReferenceId)
						{
							ReferenceIdIndex1 = id;
							break;
						}
					}
					for(int id = 0; id < nMarkers2 ; id++)
					{
						if(ids2.at(id) == ReferenceId)
						{
							ReferenceIdIndex2 = id;
							break;
						}
					}

//		Executed only once at the beginning
					if(flag)
					{
						std::cout<< "Initializing..." <<std::endl;
						if(ReferenceIdIndex1 == -1 || ReferenceIdIndex2 == -1)
						{
							cout<<"ERROR!!!"<<std::endl<< "Reference Marker is not detected!!"<<std::endl;
							break;
						}
						else
						{
// 		calculates the rotation matrix from camera frame to inertial frame
							cv::Rodrigues(rvecs1.at(ReferenceIdIndex1),RotMat12N);
							cv::Rodrigues(rvecs2.at(ReferenceIdIndex2),RotMat22N);

							float height = 7; // The height of marker from ground (For spacecrafts height = 30)
//		setting reference center as the center of reference marker
							RefCenter1 = centersInCms1.at(ReferenceIdIndex1);
							RefCenter1.x = RefCenter1.x * 242/235 - height*(5.4412/7);
							RefCenter1.y = RefCenter1.y * 242/235 - height*(2.9328/7);
							RefCenter2 = centersInCms2.at(ReferenceIdIndex2);
							RefCenter2.x = RefCenter2.x * 242/235 - height*(5.4896/7);
							RefCenter2.y = RefCenter2.y * 242/235 - height*(3.2622/7);
//		setting reference marker angle
							RefMarkerAngle1 = zAngle1.at(ReferenceIdIndex1);
							RefMarkerAngle2 = zAngle2.at(ReferenceIdIndex2);

							std::cout<< "Initialization Done"<<std::endl;
						}
						flag = false;  // setting flag to false after exectuing once
					}

//		Removing the particulars of reference marker
						if(ReferenceIdIndex1 != -1)
						{
							centersInCms1.erase(centersInCms1.begin()+ReferenceIdIndex1);
							zAngle1.erase(zAngle1.begin() + ReferenceIdIndex1);
							ids1.erase(ids1.begin() + ReferenceIdIndex1);
						}
						if(ReferenceIdIndex2 != -1)
						{
							centersInCms2.erase(centersInCms2.begin()+ReferenceIdIndex2);
							zAngle2.erase(zAngle2.begin() + ReferenceIdIndex2);
							ids2.erase(ids2.begin() + ReferenceIdIndex2);
						}

						std::vector<cv::Point2f> InertialCordinates1(0), InertialCordinates2(0), finalCordinates(0);
						std::vector<float> finalZAngle(0);

						for(std::size_t i = 0; i < centersInCms1.size(); i++)
						{
//		Converting center co-ordinates from camera frame to inertial frame
							InertialCordinates1.push_back(Cam2InertialCordinates(RefCenter1, centersInCms1.at(i), RotMat12N));
//		Calculating yaw angle w.r.t. reference marker
							zAngle1.at(i) = zAngle1.at(i) - RefMarkerAngle1;
						}
						for(std::size_t i = 0; i < centersInCms2.size(); i++)
						{
//		Converting center co-ordinates from camera frame to inertial frame
							InertialCordinates2.push_back(Cam2InertialCordinates(RefCenter2, centersInCms2.at(i), RotMat22N));
//		Calculating yaw angle w.r.t. reference marker
							zAngle2.at(i) = zAngle2.at(i) - RefMarkerAngle2;
						}

//		Final co-ordinates and yaw angle w.r.t the reference marker
							finalCordinates = FinalCordinates(ids1, ids2, InertialCordinates1, InertialCordinates2);
							finalZAngle = FinalZAngle(ids1, ids2, zAngle1, zAngle2);

						std::cout<< finalCordinates <<std::endl;

//							for(int i = 0; i<centersInCms1.size(); i++)
//							{
//								cout<< ids1.at(i)<< "\t" << InertialCordinates1.at(i)<< std::endl;
//							}
//							for(int i = 0; i<centersInCms2.size(); i++)
//							{
//								cout<< now_str() << "\t" << ids2.at(i)<< "\t" << InertialCordinates2.at(i)<< std::endl;
//							}
//							std::cout <<std::endl;

//		Publishing the data in ROS if both the space-crafts are detected
						if(finalZAngle.size() > 1)
						{
							pose1.x = finalCordinates.at(0).x;
							pose1.y = finalCordinates.at(0).y;
							pose1.z = finalZAngle.at(0);
							pose2.x = finalCordinates.at(1).x;
							pose2.y = finalCordinates.at(1).y;
							pose2.z = finalZAngle.at(1);

							pub1.publish(pose1);
							pub2.publish(pose2);
						}

						ros::spinOnce();
						rate.sleep();
				}
			cv::imshow("out1", imageCopy1);
			cv::imshow("out2", imageCopy2);
			char key = (char)
			cv::waitKey(20);
			if (key == 27)
				break;
		}
	}
}
