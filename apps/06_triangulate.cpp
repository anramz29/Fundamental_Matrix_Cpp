#include <iostream>
#include <opencv2/opencv.hpp>
#include <matching.hpp>
#include <fstream>

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

    
    // metrics from the checkerboard
    const cv::Size boardSize(7,9);
    // create checkerboard corners
    std::vector<cv::Point2f> corners_left, corners_right;
    cv::Mat gray_left, gray_right;

    // convert to grey scale
    cv::cvtColor(left, gray_left, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, gray_right, cv::COLOR_BGR2GRAY);

    // now find the chessboard corners
    bool found_l = cv::findChessboardCorners(gray_left, boardSize, corners_left);
    bool found_r = cv::findChessboardCorners(gray_right, boardSize, corners_right);

    if (!found_l || !found_r) {
        std::cerr << "chessboard not found — check board size\n";
        return 1;
    }

    // now we do corner subpix to refinment using gradients
    cv::cornerSubPix(gray_left, corners_left, 
        {11, 11}, // search window size
        {-1,-1},  // dead zone {-1, -1} means nothing
        {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001} 
        // stop after 30 iterations or when movement is less than .001
    );

    // right side
    cv::cornerSubPix(gray_right, corners_right, 
        {11,11}, 
        {-1,-1},
        {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001}
    );

    // undistort corners too
    std::vector<cv::Point2f> cl_ud, cr_ud;
    cv::undistortPoints(corners_left, cl_ud, K, dist, cv::noArray(), K);
    cv::undistortPoints(corners_right, cr_ud, K, dist, cv::noArray(), K);


    
    // triangulate corners
    cv::Mat corner_pts4D;
    cv::triangulatePoints(P1, P2, cl_ud, cr_ud, corner_pts4D);

    // lambda divides by W to get the real euclidian distance
    auto get3D = [&](int i) -> cv::Point3f {
        float w = corner_pts4D.at<float>(3, i);
        return {
            corner_pts4D.at<float>(0, i) / w,
            corner_pts4D.at<float>(1, i) / w,
            corner_pts4D.at<float>(2, i) / w
        };
    };

    // scale: one square between corner 0 and corner 1
    cv::Point3f c0 = get3D(0), c1 = get3D(1);
    double reconstructed = cv::norm(c1 - c0);
    double scale = square_mm / reconstructed;
    std::cout << "scale: " << scale << " mm/unit\n";

    // print first 5 points in mm
    std::cout << "\nfirst 5 points (mm):\n";
    for (int i = 0; i < std::min(5, (int)pts3D.size()); i++) {
        std::cout << "  ("
            << pts3D[i].x * scale << ", "
            << pts3D[i].y * scale << ", "
            << pts3D[i].z * scale << ") mm\n";
    }
    
    // cross-check: corner 0 to corner 2 should be 2 squares = 2 * square_mm
    cv::Point3f c2 = get3D(2);
    double check = cv::norm(c2 - c0) * scale;
    std::cout << "\ncross-check c0->c2: " << check
            << " mm (expected " << 2.0 * square_mm << " mm)\n";
    

        // save 2D + 3D together
    std::ofstream out("output/points3d.csv");
    out << "u,v,x,y,z\n";
    for (int i = 0; i < (int)pts3D.size(); i++) {
        out << in_right[i].x << ","   // pixel location in right image
            << in_right[i].y << ","
            << pts3D[i].x * scale << ","
            << pts3D[i].y * scale << ","
            << pts3D[i].z * scale << "\n";
    }
    
    out.close();

    std::cout << "saved points3d.csv\n";
    return 0;
}