#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

inline void computeOrbMatches(const cv::Mat& left, const cv::Mat& right,
    std::vector<cv::Point2f>& left_pts,
    std::vector<cv::Point2f>& right_pts) {


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

    // here we iterate over matches, the get queryIdx or trainIdx
    // to use it's index in kp and take .pt (x, y) position
    for (int i=0; i < matches.size(); i++){
        left_pts.push_back(kp_left[matches[i].queryIdx].pt);
        right_pts.push_back(kp_right[matches[i].trainIdx].pt);
    } 
}