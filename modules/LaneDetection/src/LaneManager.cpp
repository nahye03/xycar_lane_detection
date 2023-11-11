#include <LaneManager.hpp>

namespace XyCar
{
    LaneManager::LaneManager(PREC p_gain, PREC i_gain, PREC d_gain) : pid_controller_(p_gain, i_gain, d_gain)
    {
        // subscriber_ = node_handler_.subscribe("/usb_cam/image_raw/", 1, &LaneManager::image_callback, this);
    }

    void LaneManager::image_callback(const sensor_msgs::Image& message)
    {
        image_ = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
        cv::cvtColor(image_, image_, cv::COLOR_RGB2BGR);
        if(!image_.empty()){
            cv::imshow("image", image_);
            cv::waitKey(1);
        }
        // current_images_.push(image_processor_.process(image));
    }

    void LaneManager::run()
    {
        // if (current_images_.empty())
        //     return;

        // cv::Mat current_image = current_images_.front();
        // current_images_.pop();

        if(!image_.empty()){
            //detect rpos, lpos, and flag of stop
            std::tuple<int32_t, int32_t, bool> output_detector;
            cv::Mat canny_image = image_processor_.process(image_);
            output_detector = detector_.findPos(canny_image);
            int32_t left_pos = std::get<0>(output_detector);
            int32_t right_pos = std::get<1>(output_detector);
            bool is_stop = std::get<2>(output_detector);

            int32_t error = k_frame_width / 2 - static_cast<int32_t>((right_pos + left_pos) / 2);

            PREC angle = pid_controller_.computeAngle(error);
            (void)is_stop;
            (void)angle;

            // xycar_controller.control(angle);
        }

        
    }

}
