#ifndef LANE_DETECTION__PIDCONTROLLER_HPP
#define LANE_DETECTION__PIDCONTROLLER_HPP

// user defined header
#include "Common.hpp"

namespace XyCar{
/**
 * @details PID Controller Class
 */
class PIDController
{
public:
    /**
     * @details Construct a new PID Object.
     * @param[in] p_gain Proportional control gain.
     * @param[in] i_gain Integral control gain.
     * @param[in] d_gain Differential control gain.
     */
    PIDController(PREC p_gain, PREC i_gain, PREC d_gain);

    /**
     * @details Compute with the PID Control and return control error.
     * @param[in] error Error between the estimated x coordinates and half of the image.
     * @return XyCar::PREC
     */
    PREC computeAngle(int32_t error);

private:
    PREC proportional_gain_;
    PREC integral_gain_;
    PREC differential_gain_;

    PREC proportional_error_ = 0.0;
    PREC integral_error_ = 0.0;
    PREC differential_error_ = 0.0;
};
} // XyCar

#endif // LANE_DETECTION__PIDCONTROLLER_HPP
