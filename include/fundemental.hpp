#pragma once
#include <opencv2/opencv.hpp>


inline std::vector<cv::Point2f> normalizePoints(const std::vector<cv::Point2f>& pts, cv::Mat& T){
    double centeroidX = 0;
    double centeroidY = 0;
    int n = pts.size();
    
    // obtain centeriods
    for (int i=0; i < pts.size(); i++){
        centeroidX += pts[i].x;
        centeroidY += pts[i].y;
    }

    // centeroid corrdinates, 
    double cx = centeroidX /n;
    double cy = centeroidY /n;

    // compute the mean distance of each point comapred to centeroid
    double running_sum = 0;
    for (int i=0; i < pts.size(); i++){
        double dx = pts[i].x - cx;
        double dy = pts[i].y - cy;
        
        // distance from point to centeroid
        running_sum += std::sqrt(dx*dx + dy*dy);
    }

    // find our mean distance
    double mean_distance = running_sum / n;

    // obtain our s component
    double s = std::sqrt(2.0) / mean_distance;


    // build or T matrix eye is Identity matrix
    T = cv::Mat::eye(3,3,CV_64F);


    T.at<double>(0,0) = s;
    T.at<double>(1,1) = s;
    T.at<double>(0,2) = -s*cx;
    T.at<double>(1,2) = -s*cy;

    
    // apply T to eeach point to build normalized output
    std::vector<cv::Point2f> normalized;
    for (int i =0; i < n; i++){
        double nx = s * (pts[i].x - cx);
        double ny = s * (pts[i].y - cy);
        normalized.push_back(cv::Point2f(nx, ny));
    }
    return normalized;
}

// hand-rolled normalized 8-point algorithm.
// pure function: points in, fundamental matrix out. no loading, no drawing.
inline cv::Mat computeFundamentalMatrix(const std::vector<cv::Point2f>& pts1,
                                 const std::vector<cv::Point2f>& pts2) {
    
    // std::cout << "computeF called with " << pts1.size() << " points\n";
    cv::Mat T1, T2;
    std::vector<cv::Point2f> npts1 =normalizePoints(pts1, T1);
    std::vector<cv::Point2f> npts2 = normalizePoints(pts2, T2);
    
    int n = (int)npts1.size(); //num of correspondeces, used to iterate over
    cv::Mat A(n, 9, CV_64F); // n*9 matrix to be our A


    for (int i=0; i<n; i++){

        // define our u,v for both left and right, prime denotes right
        double u = npts1[i].x;
        double v = npts1[i].y;
        double u_p = npts2[i].x; // u_p refers to u prime
        double v_p = npts2[i].y;

        // enter these values into a row in A
        A.at<double>(i, 0) = u_p * u;
        A.at<double>(i, 1) = u_p * v;
        A.at<double>(i, 2) = u_p;
        A.at<double>(i, 3) = v_p * u;
        A.at<double>(i, 4) = v_p * v;
        A.at<double>(i, 5) = v_p;
        A.at<double>(i, 6) = u;
        A.at<double>(i, 7) = v;
        A.at<double>(i, 8) = 1;
    }
    // SVD solve Af = 0
    // the first SVD gives us svd.u, svd.w. svd.vt, the respective diagonal 
    // scale and horizontal matrix's

    cv::SVD svd(A, cv::SVD::FULL_UV);

    // we get the last row of the svd' vt matrix, as values are sorted
    // largest to smallest singular values, which is the most 'squahed'
    // direction and gives the solution Af = 0
    cv::Mat f = svd.vt.row(8);

    // now we reshape f into a 3 by 3 matrix denoted by F_hat
    cv::Mat F_hat = f.reshape(0, 3);

    // now we have to apply SVD once more to enforce rank 2
    // in more detail svd2.w are the three signular values [σ1​,σ2​,σ3​]
    // we wnat to remove σ3 (layer 3's shape) as it's the gentlest
    // way to enforce rank 2
    cv::SVD svd2(F_hat);
    cv::Mat w = svd2.w.clone();
    w.at<double>(2) = 0; // replacling σ3

    // now we just put the matrix back tougther
    cv::Mat F = svd2.u * cv::Mat::diag(w) * svd2.vt;

    // now we denormalize the matrix
    F = T2.t() * F * T1;

    return F;  // placeholder for now
}

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