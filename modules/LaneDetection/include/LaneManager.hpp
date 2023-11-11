#ifndef LANE_DETECTION__LANEMANAGER_HPP
#define LANE_DETECTION__LANEMANAGER_HPP

// system header
#include <queue>
#include <iostream>

// third party header
#include "sensor_msgs/Image.h"

// user defined header
#include "Common.hpp"
#include "ImageProcessor.hpp"
#include "LaneDetector.hpp"
#include "PIDController.hpp"
// #include "XycarController.hpp"

namespace XyCar
{
class LaneManager
{
public:
    LaneManager(PREC p_gain, PREC i_gain, PREC d_gain);

    void run();

private:
    // ros::NodeHandle node_handler_;
    // ros::Subscriber subscriber_;

    ImageProcessor image_processor_;
    LaneDetector detector_;
    PIDController pid_controller_;
    // XycarController xycar_controller;

    // std::queue <cv::Mat> current_images_;
    cv::Mat image_;

    void image_callback(const sensor_msgs::Image& message);
};
} // XyCar

#endif // LANE_DETECTION__LANEMANAGER_HPP