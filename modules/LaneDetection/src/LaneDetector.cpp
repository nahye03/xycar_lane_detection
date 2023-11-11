//
// Created by nahye on 23. 11. 7.
//

#include "LaneDetector.hpp"

namespace XyCar
{
    void LaneDetector::divideLeftRightLine(const std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& left_lines, std::vector<cv::Vec4i>& right_lines, std::vector<cv::Vec4i>& stop_lines)
    {
        constexpr double k_low_slope_threshold = 0.1;
        constexpr double k_stop_slpoe_threshold = 0.15;

        constexpr int32_t k_half_frame = k_frame_width / 2;
        constexpr int32_t k_threshold_location = k_frame_width / 5;

        for(const cv::Vec4i& line : lines)
        {
            int32_t x1 = line[0];
            int32_t y1 = line[1];
            int32_t x2 = line[2];
            int32_t y2 = line[3];

            if(x2 == x1)
                continue;

            double slope = static_cast<double>(y2 - y1) / (x2 - x1);

            if((slope < -k_low_slope_threshold) && (x1 < k_half_frame))
                left_lines.emplace_back(x1,y1,x2,y2);

            else if((slope > k_low_slope_threshold) && (x2 > k_half_frame))
                right_lines.emplace_back(x1,y1,x2,y2);

            else if((abs(slope) <= k_stop_slpoe_threshold) && (x1 > k_threshold_location) && (x2 < k_threshold_location * 4))
                stop_lines.emplace_back(x1,y1,x2,y2);
        }
    }

    void LaneDetector::findStopLine(const std::vector<cv::Vec4i> &stoplines)
    {
        if(stoplines.size() >= 2)
            state_.stop_flag_ = true;
        else
            state_.stop_flag_ = false;
    }

    void LaneDetector::calculateSlopeAndIntercept(const std::vector<cv::Vec4i>& lines, bool is_left)
    {
        double length_sum = 0.0;
        double slope_sum = 0.0;
        double intercept_sum = 0.0;

        for(const cv::Vec4i& line : lines)
        {
            int32_t x1 = line[0];
            int32_t y1 = line[1];
            int32_t x2 = line[2];
            int32_t y2 = line[3];

            if(x2 - x1 == 0)
                continue;

            int32_t diff_y = y2 - y1;
            int32_t diff_x = x2 - x1;
            double slope = static_cast<double>(diff_y) / (diff_x);
            double intercept = y1 + k_roi_frame_height - slope * x1;
            double line_length = sqrt(diff_y * diff_y) + (diff_x * diff_x);

            length_sum += line_length;
            slope_sum += slope * line_length;
            intercept_sum += intercept * line_length;
        }

        if(std::round(length_sum) != 0)
        {
            if(is_left){
                state_.left_slope_ = slope_sum / length_sum;
                state_.left_intercept_ = intercept_sum / length_sum;
            }
            else{
                state_.right_slope_ = slope_sum / length_sum;
                state_.right_intercept_ = intercept_sum / length_sum;
            }
        }
    }

    void LaneDetector::calculatePos(bool is_left)
    {
        if(is_left){
            if(std::round(state_.left_slope_) == 0 && std::round(state_.left_intercept_) == 0){
                state_.left_pos_ = -1;
            }
            else{
                state_.left_pos_ = static_cast<int32_t>((k_offset - state_.left_intercept_)/ state_.left_slope_);
                if(state_.left_pos_ < 0)
                    state_.left_pos_ = 0;
            }
        }
        else{
            if(std::round(state_.right_slope_) == 0 && std::round(state_.right_intercept_) == 0){
                state_.right_pos_ = k_frame_width + 1;
            }
            else{
                state_.right_pos_ = static_cast<int32_t>((k_offset - state_.right_intercept_)/ state_.right_slope_);
                if(state_.right_pos_ > k_frame_width)
                    state_.right_pos_ = k_frame_width;
            }
        }
    }

    void LaneDetector::refinePos()
    {
        constexpr double k_under_limit = 0.6;
        constexpr double k_upper_limit = 1.0;

        if(state_.left_pos_ < 0){
            if((state_.right_pos_ <= k_frame_width) && (k_under_limit < abs(state_.right_slope_)) && (abs(state_.right_slope_) < k_upper_limit)){
                state_.left_pos_ = state_.right_pos_ - k_lane_width;
                state_.left_slope_ = -state_.right_slope_;
                state_.left_intercept_ = k_offset - state_.left_slope_ * state_.left_pos_;
                if(state_.left_pos_ < 0) state_.left_pos_ = 0;
            }
            else {
                state_.left_pos_ = 0;
            }
        }
        else if(state_.right_pos_ > k_frame_width){
            if((state_.left_pos_ >= 0) && (k_under_limit < abs(state_.left_slope_)) && (abs(state_.left_slope_) < k_upper_limit)){
                state_.right_pos_ = state_.left_pos_ + k_lane_width;
                state_.right_slope_ = -state_.left_slope_;
                state_.right_intercept_ = k_offset - state_.right_slope_ * state_.right_pos_;
                if(state_.right_pos_ > k_lane_width) state_.right_pos_ = k_frame_width;
            }
            else {
                state_.right_pos_ = k_frame_width;
            }
        }
    }
}