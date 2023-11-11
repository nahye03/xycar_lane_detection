#include "ImageProcessor.hpp"

namespace XyCar
{
    void ImageProcessor::init()
    {
        mask_lidar_ = cv::imread("../src/mask.png", cv::IMREAD_GRAYSCALE);
    }

    cv::Mat ImageProcessor::process(const cv::Mat &frame)
    {
        frame.copyTo(cropped_frame_);
        cropped_frame_ = cropped_frame_(cv::Rect(0, k_roi_frame_height, k_frame_width, k_frame_height - k_roi_frame_height));

        // gray image
        cv::cvtColor(cropped_frame_, cropped_frame_, cv::COLOR_BGR2GRAY);

        // binarization
        cv::equalizeHist(cropped_frame_, cropped_frame_);
        cv::threshold(cropped_frame_, cropped_frame_, 65, 255, cv::THRESH_BINARY_INV);

        // lidar mask
        cv::bitwise_and(cropped_frame_, mask_lidar_(cv::Rect(0,(mask_lidar_.rows>>3)*5,mask_lidar_.cols,(mask_lidar_.rows>>3)*3)), cropped_frame_);

        // blur (gaussian)
        cv::GaussianBlur(cropped_frame_, cropped_frame_, cv::Size(), 5);

        // canny edge
        cv::Canny(cropped_frame_, cropped_frame_, 50, 150);

        return cropped_frame_;
    }

} // XyCar
