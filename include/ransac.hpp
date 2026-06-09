#pragma once
#include <opencv2/opencv.hpp>
#include <fundemental.hpp>

inline double epipolarError(const cv::Mat& F, const cv::Point2f& left_pts,
    const cv::Point2f& right_pts){
    // testing is based of xr^T * F * xl =0, we use Mat_ here
    cv::Mat xl = (cv::Mat_<double>(3,1) << left_pts.x, left_pts.y, 1.0);
    cv::Mat xr = (cv::Mat_<double>(3,1) << right_pts.x, right_pts.y, 1.0);
    // we want the product to be as close to zero as possible, main property of the
    // fundemental matrix
    cv::Mat e = xr.t() * F * xl;

    // pull out the value as or numerator
    double num = e.at<double>(0,0);

    cv::Mat Fxl = F * xl;
    cv::Mat Ftxr = F.t() * xr;

    // now we apply the sampson distance to select inliners byt geomertic cloness
    // as without it algebriac error was letting a fake rotation slip through
    double d = Fxl.at<double>(0)*Fxl.at<double>(0)
        + Fxl.at<double>(1)*Fxl.at<double>(1)
        + Ftxr.at<double>(0)*Ftxr.at<double>(0)
        + Ftxr.at<double>(1)*Ftxr.at<double>(1);

    // Sampson distance = num^2 / d  (guard against divide-by-zero)
    return (d > 1e-12) ? (num*num) / d : 0.0;
}

inline cv::Mat RansacFundamental(const std::vector<cv::Point2f>& pts_left,
                        const std::vector<cv::Point2f>& pts_right,
                        double threshold=3.0, int iterations=2000){

        // store data outside of loop
        int n = (int)pts_left.size();
        int best_inliner_count = 0;
        std::vector<int> best_inliners;
        cv::RNG rng;

        for (int iter =0; iter<iterations; iter++){
            std::vector<int> sample_idx;
            while(sample_idx.size() < 8){
                // now we apply rng uniform, radnom numb generator
                int r = rng.uniform(0, n);

                // only add if not chosen already
                bool already = false;

                // double check for no duplicates
                for (int s : sample_idx){
                    if (s==r) { already = true; break;}}
                if (!already){ sample_idx.push_back(r);}

            }

            std::vector<cv::Point2f> sample_left;
            std::vector<cv::Point2f> sample_right;
            for (int i=0; i < sample_idx.size(); i++){
                // now we store the correspondences
                sample_left.push_back(pts_left[sample_idx[i]]);
                sample_right.push_back(pts_right[sample_idx[i]]);

            }


            cv::Mat F = computeFundamentalMatrix(sample_left, sample_right);

            // here we count score across all points
            std::vector<int> inliners;
            for (int i=0; i<n; i++){
                // obtain all of the inliners
                double error = epipolarError(F, pts_left[i], pts_right[i]);
                if (error < threshold){
                    inliners.push_back(i);
                }
            }

            // save the best inliners
            if ((int)inliners.size() > best_inliner_count){
                best_inliner_count = (int)inliners.size();
                best_inliners = inliners;
            }
        };

        std::cout << "RANSAC: " << best_inliner_count << " / " << n << " inliers\n";

        if (best_inliner_count < 8) {
            std::cerr << "RANSAC failed: not enough inliers (" << best_inliner_count << ")\n";
            return cv::Mat();   // or fall back to the all-points F
        }

        // convert the best inliner idexes to left and right points
        std::vector<cv::Point2f> in_left, in_right;
        for (int idx : best_inliners){
            in_left.push_back(pts_left[idx]);
            in_right.push_back(pts_right[idx]);
        };

        // refit the best inliners
        cv::Mat Best_F = computeFundamentalMatrix(in_left, in_right);

        return Best_F;
}
