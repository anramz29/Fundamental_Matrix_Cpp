#include <iostream>
#include <opencv2/opencv.hpp>


class load_images {
public:

    cv::Mat left_img;
    cv::Mat right_img;

    load_images(const std::string& left_path, 
        const std::string& right_path){
        left_img = cv::imread(left_path);
        right_img = cv::imread(right_path);

        if (left_img.empty() || right_img.empty()){
            std::cerr << "failed to load image \n";
        }
    }
};

int main() {
    // create a load_images object — this runs the constructor with these paths
    load_images images("data/left.jpg", "data/right.jpg");

    // show each image in a window
    cv::imshow("left", images.left_img);
    cv::imshow("right", images.right_img);

    // wait for a keypress before closing (0 = wait forever)
    cv::waitKey(0);

    return 0;
}