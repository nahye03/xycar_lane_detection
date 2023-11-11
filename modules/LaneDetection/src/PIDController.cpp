#include "PIDController.hpp"

namespace XyCar
{
    PIDController::PIDController(PREC p_gain, PREC i_gain, PREC d_gain)
        : proportional_gain_(p_gain), integral_gain_(i_gain), differential_gain_(d_gain) {}

    PREC PIDController::computeAngle(int32_t error)
    {
        PREC cast_error = static_cast<PREC>(error);
        differential_error_ = cast_error - proportional_error_;
        proportional_error_ = cast_error;
        integral_error_ += cast_error;

        return proportional_gain_ * proportional_error_ + integral_gain_ * integral_error_ + differential_gain_ * differential_error_;
    }
} // XyCar