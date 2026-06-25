#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat left  = cv::imread("output/left_rect.jpg",  cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread("output/right_rect.jpg", cv::IMREAD_GRAYSCALE);
    if (left.empty() || right.empty()) {
        std::cerr << "run 07_rectify first — left_rect.jpg / right_rect.jpg not found\n";
        return 1;
    }

    cv::FileStorage fs("output/rectify.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "run 07_rectify first — rectify.yml not found\n";
        return 1;
    }
    cv::Mat Q, P1_rect, P2_rect;
    fs["Q"]       >> Q;
    fs["P1_rect"] >> P1_rect;
    fs["P2_rect"] >> P2_rect;
    fs.release();

    std::cout << "images: " << left.cols << "x" << left.rows << "\n";
    std::cout << "Q:\n" << Q << "\n";

    return 0;
}
