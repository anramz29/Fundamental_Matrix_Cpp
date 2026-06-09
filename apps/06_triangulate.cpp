#include <iostream>
#include <opencv2/opencv.hpp>
#include <matching.hpp>

int main(){
    // load in camera intrinsics
    cv::FileStorage cam_in("output/camera_calibration.yml", cv::FileStorage::READ);
    cv::FileStorage E_in("output/essential.yml", cv::FileStorage::READ);

    // add checks for both
    if (!E_in.isOpened()) {
    std::cerr << "ERROR: could not open output/essential.yml\n";
    return 1;
    }

    if (!cam_in.isOpened()) {
        std::cerr << "ERROR: could not open output/camera_calibration.yml\n";
        return 1;
    }

    // now reaload our left and right images
    cv::Mat left = cv::imread("data/left.jpg");
    cv::Mat right = cv::imread("data/right.jpg");

    if (left.empty() || right.empty()) {
        std::cerr << "failed to load images\n";
        return 1;
    }

    // read in our info
    cv::Mat R, t, K, dist;
    E_in["rotation_matrix"]          >> R;
    E_in["translation_vector"]       >> t;
    cam_in["camera_matrix"]          >> K;
    cam_in["distortion_coefficients"] >> dist;
    double square_mm;
    cam_in["square_size_mm"]         >> square_mm;

    //release
    E_in.release();
    cam_in.release();

    // match + undistort (same as we did for the essential matrix)
    std::vector<cv::Point2f> left_pts, right_pts;
    computeOrbMatches(left, right, left_pts, right_pts);

    std::vector<cv::Point2f> left_ud, right_ud;
    cv::undistortPoints(left_pts,  left_ud,  K, dist, cv::noArray(), K);
    cv::undistortPoints(right_pts, right_ud, K, dist, cv::noArray(), K);

    // inlier mask
    std::vector<uchar> inlier_mask;
    cv::findFundamentalMat(left_ud, right_ud,
        cv::FM_RANSAC, 3.0, 0.999, inlier_mask);

    std::vector<cv::Point2f> in_left, in_right;
    for (size_t i = 0; i < inlier_mask.size(); i++) {
        if (inlier_mask[i]) {
            in_left.push_back(left_ud[i]);
            in_right.push_back(right_ud[i]);
        }
    }
    std::cout << "inliers: " << in_left.size() << "\n";

    // now we create our P1 which is our left camera defined at the world origin so it's
    // the transformation matrix is I for rotation and [0,0,0] for translation
    cv::Mat P1(3, 4, CV_64F, cv::Scalar(0));
    K.copyTo(P1(cv::Rect(0, 0, 3, 3)));

    // therefore P2 horizontally concatens the rotation and translation compenets from the 
    // essentail.yml to give us oru transformation matrix that we multipy by K
    cv::Mat Rt;
    cv::hconcat(R, t, Rt);
    cv::Mat P2 = K * Rt;

    // we use trig to triangulate the points, tigiagulate points uses svd to minimize the 
    // algebraic error
    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, in_left, in_right, pts4D);

    // now we turn these into 3d coords by dividing by w the last entry in pts4D
    std::vector<cv::Point3f> pts3D;
    for (int i = 0; i < pts4D.cols; i++) {
        float w = pts4D.at<float>(3, i);
        pts3D.push_back({
            pts4D.at<float>(0, i) / w,
            pts4D.at<float>(1, i) / w,
            pts4D.at<float>(2, i) / w
        });
    }
    std::cout << "triangulated: " << pts3D.size() << " points\n";

    // TODO: use the checkerboardsin teh image to triangulate and get the scale of the movem
    // TODO: ment done





















}