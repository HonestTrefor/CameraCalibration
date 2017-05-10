//C++ Includes
#include <iostream>
#include <vector>
 
//OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
 
int main(int argc, char* argv[]){
     
    std::vector<cv::Mat> images;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Point2f> pointBuf;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    cv::Size imageSize;
    cv::Size boardSize(9,6);
    float squareSize = 2.36;
 
    //pointBuf.reserve(boardSize.height * boardSize.width * argc);
 
     
    //Check input
    if(argc < 2){
        std::cout << "Place images as function parameters" << std::endl;
        return -1;
    }
     
    cv::namedWindow("Display Window", CV_WINDOW_NORMAL);
     
    //Load images and find their dots
    for(int i = 0; i < argc-1; i++){
         
        //load images
        images.push_back(cv::imread(argv[i+1], -1));
         
        if(!images[i].data){
            std::cout << "failed to load image number " << i << std::endl;
            break;
        }
        std::cout << "Loaded image " << i << std::endl;
        cv::Mat viewGray;
        cv::cvtColor(images[i], viewGray, CV_BGR2GRAY);
        bool found = cv::findChessboardCorners( viewGray, boardSize, pointBuf );
         
        std::vector<cv::Point3f> obj;
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                obj.push_back(cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));
         
        if(found){
            cv::cornerSubPix( viewGray, pointBuf, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
             
            cv::drawChessboardCorners( images[i], boardSize, cv::Mat(pointBuf), found );
            std::cout << "Found chessboard corners" << std::endl;
             
            //Resize image to display
            cv::resize(images[i], images[i], cv::Size(images[i].cols/4, images[i].rows/4)); // to half size or even smaller
             
            cv::imshow("Display Window", images[i]);
            cv::waitKey(1);
 
            imagePoints.push_back(pointBuf);
            objectPoints.push_back(obj);
        }
    }
     
    //Find intrinsic and extrinsic camera parameters
    objectPoints.resize(imagePoints.size(),objectPoints[0]);
    imageSize.width  = images[0].cols;
    imageSize.height = images[0].rows;
     
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K4);
     
    std::cout << "Calibration Quality = " << rms << std::endl;    
    std::cout << cameraMatrix << std::endl;
     
    double fx = cameraMatrix.at<double>(0,0);
    std::cout << "Fx = " << fx << std::endl;
     
    int depthFocalLength = fx/4.05;
    std::cout << "Depth sensor focal length = " << depthFocalLength << std::endl;
     
    double percent = ((depthFocalLength/570.5) - 1)*100;
    std::cout << "Percent = " << percent << std::endl;
     
    return 0;
}