#include <iostream>
#include <opencv2/opencv.hpp>
#include <epipolar_viz.hpp>
#include <matching.hpp>

int main(){
    cv::Mat left = cv::imread("data/left.jpg");
    cv::Mat right = cv::imread("data/right.jpg");

    if (left.empty() || right.empty()) {
        std::cerr << "failed to load images\n";
        return 1;
    }

    std::vector<cv::Point2f> left_pts; 
    std::vector<cv::Point2f> right_pts;

    computeOrbMatches(left, right, left_pts, right_pts);

    if (left_pts.size() < 8) {
        std::cerr << "not enough matches after ratio test\n";
        return 1;
    }

    // calculate the fundemental matrix
    // explicit RANSAC + capture inlier mask
    std::vector<uchar> inlier_mask;
    cv::Mat f_matrix = cv::findFundamentalMat(
        left_pts, right_pts, cv::FM_RANSAC, 3.0, 0.99, inlier_mask);


    if (f_matrix.empty()) {
        std::cerr << "findFundamentalMat failed\n";
        return 1;
    }

    // keep only inliers for drawing
    std::vector<cv::Point2f> in_left, in_right;
    for (size_t i = 0; i < inlier_mask.size(); i++) {
        if (inlier_mask[i]) {
            in_left.push_back(left_pts[i]);
            in_right.push_back(right_pts[i]);
        }
    }
    
    std::cout <<"Fundemental Matrix: " << f_matrix << std::endl;
    std::cout << "inliers: " << in_left.size() << " / " << left_pts.size() << "\n";

    // plotting
    cv::Mat ocv_my_f = EpipolarViz::drawEpipolarLines(right, f_matrix, left_pts, right_pts);
    cv::imwrite("output/ocv_f_scene.png",     ocv_my_f);

    cv::imshow("Open CV F",     ocv_my_f);
    cv::waitKey(0);
    
    
    return 0;
}