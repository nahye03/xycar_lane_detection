#ifndef LANE_DETECTION__DRAW_HPP
#define LANE_DETECTION__DRAW_HPP

#include "opencv2/opencv.hpp"
#include "Common.hpp"

namespace XyCar
{
    /* @details  Draw a line with slope on 'frame'.
    * @param[out] frame
    * @param[in]  slope  The slope of a lane.
    * @param[in]  intercept  The intercept of a lane.
    * @param[in]  color  The color of lane.
    * @return  void
    */
    void drawLineWithSlope(cv::Mat& frame, double slope, double intercept, const cv::Scalar& color)
    {
        if(slope == 0) return;
        int32_t y1 = k_frame_height;
        int32_t y2 = std::round(y1>>1);
        int32_t x1 = std::round((y1 - intercept) / slope);
        int32_t x2 = std::round((y2 - intercept) / slope);
        cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), color, 2, cv::LINE_8);
    }

    /* @details  Draw a line using Coordination on 'frame'.
    * @param[out] frame
    * @param[in]  point1  The first point defining the line.
    * @param[in]  point2  The second point defining the line.
    * @param[in]  color  The color of lane.
    * @return  void
    */
    void drawLineWithPoints(cv::Mat& frame, cv::Point point1, cv::Point point2, const cv::Scalar& color)
    {
        cv::line(frame, point1, point2, color, 2, cv::LINE_8);
    }

    /* @details Draw a rectangle on frame.
    * @param[out] frame
    * @param[in]  pos  The x-coordinate position of the rectangle.
    * @return  void
    */
    void drawRectangle(cv::Mat& frame, int32_t pos)
    {
        cv::rectangle(frame, cv::Rect(cv::Point(pos - 5, k_offset - 5),cv::Point(pos + 5, k_offset - 5)), cv::Scalar(0, 255, 0));
    }

} // XyCar

#endif // LANE_DETECTION__DRAW_HPP