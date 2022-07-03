#include <opencv2/opencv.hpp>
#include "CalibrationIO.h"

int main(int argc, char* argv[]) {

    //cv::namedWindow("Left");
    //cv::namedWindow("Right");

    char fname[255];

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints1;
    std::vector<std::vector<cv::Point2f>> imagePoints2;
    cv::Size imageSize(1920, 1080); // in pixels
    cv::Size patternSize(10, 5); // in squares
    cv::Mat K1;
    cv::Mat K2;
    std::vector<double> distCoeffs1;
    std::vector<double> distCoeffs2;
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<cv::Point3f> checkerboardPattern;

    for (int y = 0; y < patternSize.height; ++y) {
        for (int x = 0; x < patternSize.width; ++x) {
            checkerboardPattern.push_back(cv::Point3f(x * 47, y * 47, 0));
        }
    }
    for (int i = 457; i < 476; ++i) {
        std::vector<cv::Point2f> corners;

        sprintf_s(fname, "../CalibrationLeft/DSCF%04d_L.JPG", i);
        cv::Mat imageL = cv::imread(fname);
        cv::findChessboardCorners(imageL, patternSize, corners);
        imagePoints1.push_back(corners);
        objectPoints.push_back(checkerboardPattern);


        //cv::imshow("Left", imageL);
        //cv::waitKey(10);
        
    }

    double err1 = cv::calibrateCamera(objectPoints, imagePoints1, imageSize, K1, distCoeffs1, rvecs, tvecs);

    std::cout << "Reprojection err camera 1: " << err1 << std::endl;
    std::cout << "Calibation matrix camera 1: " << std::endl << K1 << std::endl;

    for (int j = 457; j < 476; j++) {
        std::vector<cv::Point2f> corners;

        sprintf_s(fname, "../CalibrationRight/DSCF%04d_R.JPG", j);
        cv::Mat imageR = cv::imread(fname);
        cv::findChessboardCorners(imageR, patternSize, corners);
        //cv::imshow("Right", imageR);
        //cv::waitKey(100);
        imagePoints2.push_back(corners);
    }
    double err2 = cv::calibrateCamera(objectPoints, imagePoints2, imageSize, K2, distCoeffs2, rvecs, tvecs);

    std::cout << "Reprojection err camera 2: " << err2 << std::endl;
    std::cout << "Calibation matrix camera 2: " << std::endl << K2 << std::endl;

    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;

    cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2, K1, distCoeffs1, K2, distCoeffs2, imageSize, R, T, E, F);

    std::cout << "Stereo Calibration Complete" << std::endl;
    std::cout << "Rotation:" << std::endl;
    std::cout << R << std::endl;
    std::cout << "Translation:" << std::endl;
    std::cout << T << std::endl;
    std::cout << "Essential Matrix:" << std::endl;
    std::cout << E << std::endl;
    std::cout << "Fundamental Matrix:" << std::endl;
    std::cout << F << std::endl;

    saveStereoCalibration("../s_cal.txt", K1, distCoeffs1, K2, distCoeffs2, R, T);

    return 0;
}

