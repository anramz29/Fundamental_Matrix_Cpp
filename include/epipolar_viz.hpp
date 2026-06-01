#pragma once
#include <opencv2/opencv.hpp>

class EpipolarViz{
public:
    static cv::Mat drawEpipolarLines(const cv::Mat& image,
        const cv::Mat& F,
        const std::vector<cv::Point2f>& pts_left,
        const std::vector<cv::Point2f>& pts_right) {
        
        
        std::vector<cv::Vec3f> lines;
        cv::computeCorrespondEpilines(pts_left, 1, F, lines);

        cv::Mat out = image.clone();
        for (int i = 0; i < (int)lines.size() && i < 30; i++) {
            float a = lines[i][0], b = lines[i][1], c = lines[i][2];
            cv::Point p0(0, (int)(-c / b));
            cv::Point p1(out.cols, (int)(-(c + a * out.cols) / b));
            cv::line(out, p0, p1, cv::Scalar(0, 255, 0), 1);
            cv::circle(out, pts_right[i], 4, cv::Scalar(0, 0, 255), -1);
        }
        return out;
    }
};