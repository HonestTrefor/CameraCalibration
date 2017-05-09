#include <iostream>
#include "opencv2\core.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\calib3d.hpp"
#include "opencv2\imgproc.hpp"
#include <vector>


int main(int argc, char* argv[]){

	std::vector<cv::Mat> images;
	images.reserve(argc - 1);
	cv::Size boardSize(9,6);

	std::vector<std::vector<cv::Point2f>> imagePoints;
	std::vector<std::vector<cv::Point3f>> objectPoints;
	std::vector<cv::Point2f> pointBuf;

	int squareSize = 20;

	pointBuf.reserve(boardSize.height * boardSize.width * argc);

	//Check input
	if(argc < 2){
		std::cout << "Place images as function parameters" << std::endl;
	}

	//Load images and find their dots 
	for(int i = 0; i < argc-1; i++){
		//load images 
		images.push_back(cv::imread(argv[i+1], -1));

		if(!images[i].data){
			std::cout << "failed to load image number " << i << std::endl;
			break;
		}

		bool found = cv::findChessboardCorners( images[i], boardSize, pointBuf );

		std::vector<cv::Point3f> obj;
		for( int i = 0; i < boardSize.height; ++i )
			for( int j = 0; j < boardSize.width; ++j )
				obj.push_back(cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));

		if(found){
			cv::Mat viewGray;
			cv::cvtColor(images[i], viewGray, CV_BGR2GRAY);
			cv::cornerSubPix( viewGray, pointBuf, cv::Size(9,6), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			cv::drawChessboardCorners( images[i], boardSize, cv::Mat(pointBuf), found );
			cv::namedWindow("Display Window", CV_WINDOW_NORMAL);
			cv::imshow("Display Window", images[i]);
			cv::waitKey();

			imagePoints.push_back(pointBuf);
			objectPoints.push_back(obj);
		}
	}

	//Find intrinsic and extrinsic camera parameters
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	std::vector<cv::Mat> rvecs, tvecs;

	objectPoints.resize(imagePoints.size(),objectPoints[0]);
	cv::Size imageSize;
	imageSize.width = images[0].cols;
	imageSize.height = images[0].rows;

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K5);

	//double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K4);


	return 0;
}