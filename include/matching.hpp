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
    cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);

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

    // knnMatch (k=2) so we have the two nearest neighbours for the ratio test.
    // do NOT set crossCheck on the matcher here — it conflicts with knnMatch.
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(desc_left, desc_right, knn, 2);

    // Lowe's ratio test: keep a match only when the best neighbour is clearly
    // closer than the second best. discards ambiguous matches before they
    // ever reach the fundamental-matrix solvers.
    for (auto& m : knn) {
        if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance) {
            left_pts.push_back(kp_left[m[0].queryIdx].pt);
            right_pts.push_back(kp_right[m[0].trainIdx].pt);
        }
    }
    std::cout << "good matches: " << left_pts.size() << "\n";
}