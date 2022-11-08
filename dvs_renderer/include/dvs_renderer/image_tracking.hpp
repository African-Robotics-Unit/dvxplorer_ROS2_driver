#ifndef IMAGE_TRACKING_HPP
#define IMAGE_TRACKING_HPP

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "dvxplorer_interfaces/msg/event.hpp"
#include "dvxplorer_interfaces/msg/event_array.hpp"

// Camera/ImageTransport includes
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"

// OpenCV2 includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>

namespace dvs_renderer {

/**
 * @class ImageTracking
 * Track frames over 1 sec, then project all events during this time into
 * the latest frame & output the accumulated events per pixel.
 * This requires the scene to be planar, and for best results, the camera motion
 * is perpendicular to the plane's surface normal.* 
 */
class ImageTracking {
public:
    ImageTracking(rclcpp::Node::SharedPtr node);
    void eventsCallback(const dvxplorer_interfaces::msg::EventArray::SharedPtr & msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr & msg);

private:
    rclcpp::Node::SharedPtr node;

    void render();
    void reset();

    struct ImgData {
        cv::Mat img;
        rclcpp::Time time;
        std::vector<cv::Point2f> points;
        cv::Mat homography;                 // homography from previous image to this
        cv::Mat homography_accumulated;     // homography from previous image to the second last
        cv::Point2f translation;            // mean translation, previous to this
    };

    std::vector<ImgData> images_;
    std::vector<dvxplorer_interfaces::msg::Event> events_;

    double start_time_;

    cv::Mat edges_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub_events_edges_;

}; // class ImageTracking

} // namespace dvs_renderer

#endif