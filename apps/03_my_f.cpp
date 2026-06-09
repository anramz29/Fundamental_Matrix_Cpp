#include <iostream>
#include <opencv2/opencv.hpp>
#include <epipolar_viz.hpp>
#include <fundemental.hpp>
#include <ransac.hpp>
#include <matching.hpp>


int main() {
    cv::Mat left = cv::imread("data/left.jpg");
    cv::Mat right = cv::imread("data/right.jpg");

    if (left.empty() || right.empty()) {
        std::cerr << "failed to load images\n";
        return 1;
    }

    std::vector<cv::Point2f> left_pts; 
    std::vector<cv::Point2f> right_pts;

    computeOrbMatches(left, right, left_pts, right_pts);

    // we compute find our fundemntal matrixs 
    // we test vs the open cv example
    cv::Mat my_F = computeFundamentalMatrix(left_pts, right_pts);

    cv::Mat ransac_F = RansacFundamental(left_pts, right_pts, 9.0, 2000);
    ransac_F /= ransac_F.at<double>(2,2);
    std::cout << "ransac F:\n" << ransac_F << "\n";

    cv::FileStorage fs_out("output/fundamental.yml", cv::FileStorage::WRITE);
    if (!fs_out.isOpened()) {
        std::cerr << "ERROR: could not open output/fundamental.yml\n";
        return 1;
    }
    fs_out << "fundamental_matrix" << ransac_F;
    fs_out.release();
    std::cout << "Saved ransac_F to output/fundamental.yml\n";


    // plotting
    cv::Mat vis_my_f = EpipolarViz::drawEpipolarLines(right, my_F, left_pts, right_pts);
    cv::Mat vis_ransac_f = EpipolarViz::drawEpipolarLines(right, ransac_F, left_pts, right_pts);
    

    cv::imwrite("output/my_f_scene.png",     vis_my_f);
    cv::imwrite("output/ransac_f_scene.png", vis_ransac_f);

    cv::imshow("my F",     vis_my_f);
    cv::imshow("ransac F", vis_ransac_f);
    cv::waitKey(0);

    return 0;
}