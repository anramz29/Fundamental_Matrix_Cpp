#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

int main(){
    const cv::Size boardSize(7,9);

    // size of squares
    const float squareSize = 25.4f;
    const std::string imageDir = "data/calibration_imgs";
    const std::string outputFile = "camera_calibation.yml";

    // now we build our ground truth, real world coords of the chessboard
    std::vector<cv::Point3f> objp;
    for (int i=0; i < boardSize.height; i++){
        for (int c=0; c < boardSize.width; c++){
            objp.push_back(cv::Point3f(c* squareSize, i* squareSize, 0));
        }
    }

    // create two nested vectors
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints;
    cv::Size imageSize;

    // iterate over the images in the directory
    for (const auto& entry : fs::directory_iterator(imageDir)){
        // convert it to standard string so that opencv's imread can obtain the image
        std::string path = entry.path().string();

        // read the image and make sure it loads
        cv::Mat img = cv::imread(path);
        if (img.empty()) { std::cerr << "Could not load: " << path << "\n"; continue; }

        // create grey scale and convert images to grey scale
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        imageSize = gray.size();

        // empty list that will be filled with the 2D pixel positions of 
        // every detected inner corner
        std::vector<cv::Point2f> corners;

        bool found = cv::findChessboardCorners(gray, boardSize, corners,
            // uses a local thershold that adapts to different images
            cv::CALIB_CB_ADAPTIVE_THRESH |
            // normalizes image brighness before processing
            cv::CALIB_CB_NORMALIZE_IMAGE 
        );

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
        
            objPoints.push_back(objp);
            imgPoints.push_back(corners);
        
            cv::drawChessboardCorners(img, boardSize, corners, found);
            cv::imshow("Corners", img);
            cv::waitKey(200);
        
            std::cout << "OK: " << path << "\n";
        } else{
                // now we push back our objp to objectPoints and corner ect
                std::cout << "SKIP (corners not found): " << path << "\n";
        }
    }
    cv::destroyAllWindows();

    // double check that 15 of the images are valid
    if (objPoints.size() < 15) {
        std::cerr << "Need at least 3 valid images. Got: " << objPoints.size() << "\n";
        return 1;
    }

    // run calibration first create our 3 by 3 matrix identity matrix
    cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
    // create a empty matrix for the distrotion coefficients
    cv::Mat distCoeffs  = cv::Mat::zeros(8,1, CV_64F);
    // create roation (rodrigoues) and translation vector
    std::vector<cv::Mat> rvecs, tvecs;

    // get or reprojection error in pixels (MAIN Event!)
    double rms = cv::calibrateCamera(
        objPoints, imgPoints, imageSize,
        cameraMatrix, distCoeffs,
        rvecs, tvecs,
        cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5  // fix unused distortion terms
    );


    std::cout << "\nCalibration complete.\n";
    std::cout << "RMS reprojection error: " << rms << " px\n";
    std::cout << "Camera matrix:\n"  << cameraMatrix << "\n";
    std::cout << "Distortion coefficients:\n" << distCoeffs.t() << "\n";

    // --- Save results ---
    cv::FileStorage fs_out(outputFile, cv::FileStorage::WRITE);
    fs_out << "image_width"           << imageSize.width;
    fs_out << "image_height"          << imageSize.height;
    fs_out << "square_size_mm"        << squareSize;
    fs_out << "camera_matrix"         << cameraMatrix;
    fs_out << "distortion_coefficients" << distCoeffs;
    fs_out.release();

    std::cout << "Saved to: " << outputFile << "\n";

    // --- Demo: undistort one image ---
    cv::Mat sample = cv::imread(fs::directory_iterator(imageDir)->path().string());
    cv::Mat undistorted;
    cv::undistort(sample, undistorted, cameraMatrix, distCoeffs);
    cv::imwrite("undistorted_sample.jpg", undistorted);
    std::cout << "Undistorted sample saved.\n";

    return 0;
}