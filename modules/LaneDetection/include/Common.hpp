#ifndef LANE_DETECTION__COMMON_HPP
#define LANE_DETECTION__COMMON_HPP

#include <cstdint>

namespace XyCar
{
    using PREC = double;

    constexpr uint32_t k_frame_width = 640;
    constexpr uint32_t k_frame_height = 480;
    constexpr uint32_t k_roi_frame_height = (k_frame_height>>3)*5;
    constexpr uint32_t k_lane_width = 490;
    constexpr uint32_t k_offset = 400;

    /**
     * @details  A structure that represents the state of the lanes.
     */
    struct State
    {
        PREC left_slope_;
        PREC left_intercept_;
        PREC right_slope_;
        PREC right_intercept_;
        PREC left_pos_;
        PREC right_pos_;
        bool stop_flag_;
    };
} // XyCar

#endif // LANE_DETECTION__COMMON_HPP
