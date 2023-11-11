#ifndef LANE_DETECTION__KALMANFILTER_HPP
#define LANE_DETECTION__KALMANFILTER_HPP

#include "Common.hpp"
#include "opencv2/opencv.hpp"

namespace XyCar
{
/**
 * @details Class of the Kalman filter.
 */
class KalmanFilter
{
    KalmanFilter() = default;

    /**
     * @details Initialize objects for applying the Kalman filter.
     * @return void
     */
    void init();

    /**
     * @details Perform the prediction steps of the Kalman filter. \n
     *          Predicts the state vector using a given 'slope' and 'intercept'. \n
     *          Then, calculate the predicted covariance using the transition matrix and the covariance matrix. \n
     * @param[in] slope The slope of lane.
     * @param[in] intercept The intercept of lane.
     * @return void
     */
    void predict(PREC slope, PREC intercept);

    /**
     * @details Update the internal state of a Kalman filter given the 'average slope', 'average intercept' and 'position of lane'. \n
     *          The Kalman gain is computed, and the state matrix and covariance matrix are updated accordingly. \n
     *          If the position is less than or equal to 0, or greater than or equal to 640, the state matrix is updated with a measurement
     *          matrix derived from the average slope and intercept. \n
     * @param[in] avg_slope The average slope of the lane.
     * @param[in] avg_intercept The intercept slope of the lane.
     * @param[in] pos The position of the lane. (x coordinate)
     * @return void
     */
    void update(PREC avg_slope, PREC avg_intercept, PREC pos);

    /**
     * @details Get the state matrix. \n
     * When you want to receive the slope and intercept, \n
     * you receive it using the cv::Mat::at() method. \n
     * ex) slope = state_mat.at<PREC>(0, 0); \n
     * ex) intercept = state_mat.at<PREC>(2, 0); \n
     * @return const Mat&
     */
    const cv::Mat& get_state() const;

private:
    PREC slope_derivative_;
    PREC intercept_derivative_;
    cv::Mat state_matrix_;  // x: 상태 추정치
    cv::Mat transition_matrix_;  // A: 상태 변환 행렬
    cv::Mat measurement_matrix_;  // H: 측정 행렬
    cv::Mat process_noise_matrix_;  // Q: 과정 노이즈 공분산
    cv::Mat measurement_noise_matrix_;  // R: 측정 노이즈 공분산
    cv::Mat covariance_matrix_;  // P: 오차 공분산
    cv::Mat kalman_gain_;  // K: 칼만 이득
};
} // XyCar

#endif // LANE_DETECTION__KALMANFILTER_HPP
