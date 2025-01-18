// Import the aruco module in OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

int main(int argc, char **argv)
{

	// Creating aruco marker

	cv::Mat markerImage;
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
	cv::aruco::generateImageMarker(dictionary, 0, 300, markerImage, 1);
	cv::imwrite("marker0.png", markerImage);
}