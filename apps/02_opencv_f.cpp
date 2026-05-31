#include <iostream>
#include <opencv2/opencv.hpp>

int main(){
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

    // calculate the fundemental matrix
    cv::Mat f_matrix = cv::findFundamentalMat(left_pts, right_pts);


    std::cout <<"Fundemental Matrix: " << f_matrix << std::endl;
    
    // --- compute epipolar lines in the RIGHT image for points from the LEFT ---
    // each line comes back as 3 numbers (a, b, c) meaning a*x + b*y + c = 0
    std::vector<cv::Vec3f> lines_right;
    cv::computeCorrespondEpilines(left_pts, 1, f_matrix, lines_right);

    // draw them on a copy of the right image
    cv::Mat right_with_lines = right.clone();
    for (int i = 0; i < (int)lines_right.size() && i < 30; i++) {  // first 30 only, to avoid clutter
        float a = lines_right[i][0];
        float b = lines_right[i][1];
        float c = lines_right[i][2];
        // a line a*x+b*y+c=0 crosses the image; find its endpoints at x=0 and x=width
        cv::Point p0(0, (int)(-c / b));
        cv::Point p1(right.cols, (int)(-(c + a * right.cols) / b));
        cv::line(right_with_lines, p0, p1, cv::Scalar(0, 255, 0), 1);
        // also draw the corresponding point in the right image
        cv::circle(right_with_lines, right_pts[i], 4, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("epipolar lines (right image)", right_with_lines);
    cv::waitKey(0);
    
    return 0;
}