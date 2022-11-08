#ifndef DVS_RENDERER_H_
#define DVS_RENDERER_H_

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "dvxplorer_interfaces/msg/event.hpp"
#include "dvxplorer_interfaces/msg/event_array.hpp"

// Camera/ImageTransport includes
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"

#include "dvs_renderer/image_tracking.hpp"

// OpenCV2 includes
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>


namespace dvs_renderer {

class Renderer {
public:
    Renderer(rclcpp::Node::SharedPtr node);
    virtual ~Renderer() = default;

private:
    rclcpp::Node::SharedPtr node_;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr & msg) const;
    void eventsCallback(const dvxplorer_interfaces::msg::EventArray::SharedPtr& msg) const;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr& msg);

    void publishStarts();

    bool got_camera_info_;
    bool color_image_;
    cv::Mat camera_matrix_, dist_coeffs_;

    rclcpp::Subscription<dvxplorer_interfaces::msg::EventArray>::SharedPtr events_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    image_transport::Publisher image_pub_;
    image_transport::Publisher undistorted_image_pub_;

    image_transport::Subscriber image_sub_;
    cv::Mat last_image_;
    bool used_last_image_;

    struct EventStats {
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr events_mean_[2]; /**< event stats output */
        int events_counter_[2]; /**< event counters for on/off events */
        double events_mean_lasttime_;
        double dt;
    };
    EventStats event_stats_[2]; /**< event statistics for 1 and 5 sec */

    enum DisplayMethod
    {
        GRAYSCALE, RED_BLUE
    } display_method_;

    ImageTracking image_tracking_;   

}; // class renderer

} // namespace dvs_renderer

#endif //DVS_RENDERER_H_