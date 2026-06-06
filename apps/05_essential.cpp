#include <iostream>
#include <opencv2/opencv.hpp>
#include <matching.hpp>

int main(){
    cv::FileStorage f_in("output/fundamental.yml", cv::FileStorage::READ);
    cv::FileStorage cam_in("output/camera_calibration.yml", cv::FileStorage::READ);

    // add checks for both
    if (!f_in.isOpened()) {
        std::cerr << "ERROR: could not open output/fundamental.yml\n";
        return 1;
    }

    if (!cam_in.isOpened()) {
        std::cerr << "ERROR: could not open output/camera_calibration.yml\n";
        return 1;
    }

    // intialize of F and K
    cv::Mat F, K;

    // pull the file from f >> is a stream extractor operator
    f_in["fundamental_matrix"] >> F; 
    cam_in["camera_matrix"] >> K;

    // intalize our essential matrix
    cv::Mat E;

    E = K.t() * F * K;

    std::cout << "Essent Matrix: " << E << std::endl;

    // get our two possible rotations and traslation
    // Interesting: The two rotations come from SVD: there are mathematically two valid ways to factor E into 
    // rotation + translation that both satisfy the algebra, plus the translation could be negative giving us 
    // 4 possible solutions. 
    // cv::Mat R1, R2, t;

    // cv::decomposeEssentialMat(E, R1, R2, t);

    // std::cout << "Rotation Matrix 1: " << R1 << "\n" << "Rotation Matrix 2: " << R2 << std::endl;
    // std::cout << "translation vector: " << t << std::endl;

    // now reaload our left and right images
    cv::Mat left = cv::imread("data/left.jpg");
    cv::Mat right = cv::imread("data/right.jpg");

    if (left.empty() || right.empty()) {
        std::cerr << "failed to load images\n";
        return 1;
    }

    std::vector<cv::Point2f> left_pts; 
    std::vector<cv::Point2f> right_pts;

    computeOrbMatches(left, right, left_pts, right_pts);

    cv::Mat R_true, t_true;
    cv::recoverPose(E, left_pts, right_pts, K, R_true, t_true);

    std::cout << "Rotation Matrix: " << R_true << std::endl;
    std::cout << "translation vector: " << t_true << std::endl;

    // --- Save results ---
    const std::string outputFile = "output/essential.yml";

    cv::FileStorage fs_out(outputFile, cv::FileStorage::WRITE);
    fs_out << "essential_matrix"           << E;
    fs_out << "rotation_matrix"           << R_true;
    fs_out << "translation_vector"         << t_true;
    fs_out.release();
    

    return 0;

}