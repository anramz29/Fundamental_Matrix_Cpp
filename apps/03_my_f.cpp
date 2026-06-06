#include <iostream>
#include <opencv2/opencv.hpp>
#include <epipolar_viz.hpp>
#include <fundemental.hpp>


int main() {
    cv::Mat left = cv::imread("data/left.jpg");
    cv::Mat right = cv::imread("data/right.jpg");

    if (left.empty() || right.empty()) {
        std::cerr << "failed to load images\n";
        return 1;
    }

    // create the orb detector create() specifically
    // gives us a smartpointer (cv::Ptr) therefore 
    // cv:Ptr<cv::Orb> type that holds orb and cv::ORB::create()
    // is "make me one"
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    // detect key points, 
    std::vector<cv::KeyPoint> kp_left, kp_right; // where we put features
    cv::Mat desc_left, desc_right; // the descriptions of the features
    

    // because orb is a pointer we use -> to call method on pointer,
    // kp left is the keypoint, and the desc_left is thedescriptor 
    // so the function writes into them
    orb->detectAndCompute(left, cv::noArray(), kp_left, desc_left);
    orb->detectAndCompute(right, cv::noArray(), kp_right, desc_right);

    // print the count of the keypoint vectors
    std::cout << "left features:  " << kp_left.size()  << "\n";
    std::cout << "right features: " << kp_right.size() << "\n";

    // BF refers to brute force matching, and NORM_HAMMING is the distance
    // measure for orb specfically
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    // matches are a pair of indices into keypoint lists
    //DMatch ≈ (queryIdx, trainIdx)   
    std::vector<cv::DMatch> matches;
    matcher.match(desc_left, desc_right, matches);

    // print matches
    std::cout << "matches: " << matches.size() << "\n";

    // vectors of two floats because it's a 2D location
    std::vector<cv::Point2f> left_pts;
    std::vector <cv::Point2f> right_pts;
    
    // here we iterate over matches, the get queryIdx or trainIdx
    // to use it's index in kp and take .pt (x, y) position
    for (int i=0; i < matches.size(); i++){
        left_pts.push_back(kp_left[matches[i].queryIdx].pt);
        right_pts.push_back(kp_right[matches[i].trainIdx].pt);
    } 
    
    // we compute find our fundemntal matrixs 
    // we test vs the open cv example
    cv::Mat my_F = computeFundamentalMatrix(left_pts, right_pts);

    cv::Mat ransac_F = RansacFundamental(left_pts, right_pts);
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