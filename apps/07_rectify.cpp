#include <iostream>
#include <opencv2/opencv.hpp>

int main(){
    // load calibration and pose
    cv::FileStorage cam_in("output/camera_calibration.yml", cv::FileStorage::READ);
    cv::FileStorage E_in("output/essential.yml", cv::FileStorage::READ);

    if (!cam_in.isOpened() || !E_in.isOpened()) {
        std::cerr << "failed to open yml files\n";
        return 1;
    }

    cv::Mat K, dist, R, t;
    cam_in["camera_matrix"]           >> K;
    cam_in["distortion_coefficients"] >> dist;
    E_in["rotation_matrix"]           >> R;
    E_in["translation_vector"]        >> t;
    cam_in.release();
    E_in.release();

    // load images
    cv::Mat left  = cv::imread("data/left.jpg");
    cv::Mat right = cv::imread("data/right.jpg");
    if (left.empty() || right.empty()) {
        std::cerr << "failed to load images\n";
        return 1;
    }

    cv::Size imgSize = left.size();

    // stereoRectify computes the rotation matrices R1, R2 that 
    // rotate each camera so their image planes are coplanar,
    // and the new projection matrices P1_rect, P2_rect.
    // Q is the disparity-to-depth mapping matrix for later.
    cv::Mat R1, R2, P1_rect, P2_rect, Q;
    cv::stereoRectify(
        K, dist,       // left camera intrinsics
        K, dist,       // right camera intrinsics (same camera)
        imgSize,
        R, t,
        R1, R2,        // output rotation for each camera
        P1_rect,       // output projection matrix left
        P2_rect,       // output projection matrix right
        Q,             // output disparity-to-depth matrix
        cv::CALIB_ZERO_DISPARITY,
        -1,            // alpha=-1 crops to valid pixels only
        imgSize
    );

    // build the rectification maps — these are lookup tables
    // that say "pixel (u,v) in the rectified image came from 
    // pixel (map_x[u,v], map_y[u,v]) in the original image"
    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(K, dist, R1, P1_rect,
        imgSize, CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(K, dist, R2, P2_rect,
        imgSize, CV_32FC1, map2x, map2y);

    // apply the maps
    cv::Mat left_rect, right_rect;
    cv::remap(left,  left_rect,  map1x, map1y, cv::INTER_LINEAR);
    cv::remap(right, right_rect, map2x, map2y, cv::INTER_LINEAR);

    // save rectified images
    cv::imwrite("output/left_rect.jpg",  left_rect);
    cv::imwrite("output/right_rect.jpg", right_rect);

    // save Q for the disparity step
    cv::FileStorage fs_out("output/rectify.yml", cv::FileStorage::WRITE);
    fs_out << "Q" << Q;
    fs_out << "P1_rect" << P1_rect;
    fs_out << "P2_rect" << P2_rect;
    fs_out.release();

    std::cout << "saved left_rect.jpg, right_rect.jpg, rectify.yml\n";
    std::cout << "Q:\n" << Q << "\n";

    // visual check — draw horizontal lines across both images
    // if rectification is correct, features should lie on the same
    // horizontal line in both images
    cv::Mat canvas;
    cv::hconcat(left_rect, right_rect, canvas);
    for (int y = 0; y < canvas.rows; y += 100) {
        cv::line(canvas, {0, y}, {canvas.cols, y},
                 cv::Scalar(0, 255, 0), 1);
    }
    cv::imwrite("output/rectify_check.png", canvas);
    std::cout << "saved rectify_check.png — check that features align on horizontal lines\n";

    return 0;
}