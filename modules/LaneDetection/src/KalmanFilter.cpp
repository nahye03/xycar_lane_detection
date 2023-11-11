#include "KalmanFilter.hpp"

namespace XyCar
{
    void KalmanFilter::init()
    {
        constexpr PREC dt = 1.0 / 30;
        slope_derivative_ = -0.015;
        intercept_derivative_ = -3.0;

        transition_matrix_ = (cv::Mat_<double>(4, 4) << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1);
        measurement_matrix_ = (cv::Mat_<double>(2,4) << 1, 0, 0, 0, 0, 0, 1, 0);
        process_noise_matrix_ = cv::Mat::eye(4, 4, CV_64F);
        measurement_noise_matrix_ = (cv::Mat_<double>(2, 2) << 50, 0, 0, 50);
        covariance_matrix_ = 100 * cv::Mat::eye(4, 4, CV_64F);
    }

    void KalmanFilter::predict(PREC slope, PREC intercept)
    {
        state_matrix_ = transition_matrix_ * (cv::Mat_<double>(4,1) << slope, slope_derivative_, intercept, intercept_derivative_);
        static const auto transition_matrix_t = transition_matrix_.t();
        covariance_matrix_ = transition_matrix_ * covariance_matrix_ * transition_matrix_t + process_noise_matrix_;
    }

    void KalmanFilter::update(PREC avg_slope, PREC avg_intercept, PREC pos)
    {
        // kalman gain
        static const auto measurement_matrix_t = measurement_matrix_.t();
        kalman_gain_ = covariance_matrix_ * measurement_matrix_t * (measurement_matrix_ * covariance_matrix_ * measurement_matrix_t + measurement_noise_matrix_).inv();

        if (pos <= 0 || pos >= 640)
        {
            cv::Mat measurement = (cv::Mat_<double>(2, 1) << avg_slope, avg_intercept);
            state_matrix_ += kalman_gain_ * (measurement - measurement_matrix_ * state_matrix_);
        }

        // Update covariance
        covariance_matrix_ -= kalman_gain_ * measurement_matrix_ * covariance_matrix_;
    }

    const cv::Mat& KalmanFilter::get_state() const
    {
        return state_matrix_;
    }
} // XyCar
