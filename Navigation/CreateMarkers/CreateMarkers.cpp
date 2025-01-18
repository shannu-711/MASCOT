// Import the aruco module in OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

int main(int argc, char **argv)
{

	// Creating aruco marker
	float markerSizeInPixels = 220;

	cv::Mat markerImage;
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
	cv::aruco::generateImageMarker(dictionary, 0, markerSizeInPixels, markerImage, 1);
	cv::imwrite("marker0.png", markerImage);
	cv::aruco::generateImageMarker(dictionary, 1, markerSizeInPixels, markerImage, 1);
	cv::imwrite("marker1.png", markerImage);
	cv::aruco::generateImageMarker(dictionary, 7, markerSizeInPixels, markerImage, 1);
	cv::imwrite("marker7.png", markerImage);
}