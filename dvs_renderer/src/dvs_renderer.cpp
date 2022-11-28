
#include "dvs_renderer/dvs_renderer.hpp"

namespace dvs_renderer{

Renderer::Renderer()
: Node("dvs_renderer_node"), image_tracking_(this->now().seconds())
{   
    this->got_camera_info_ = false;

    // get parameters of display method
    std::string display_method_str;
    this->declare_parameter("display_method", "");
    this->get_parameter("display_method", display_method_str);
    this->display_method_ = (display_method_str == std::string("grayscale")) ? GRAYSCALE : RED_BLUE;
    
    this->declare_parameter("color", false);
    this->get_parameter("color", this->color_image_);
    this->used_last_image_ = false;

    // Subscribers and publishers
    this->events_sub_ = this->create_subscription<dvxplorer_interfaces::msg::EventArray>(
            "dv/events",
            1,
            std::bind(&Renderer::eventsCallback, this ,std::placeholders::_1)
    );

    this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info",
            1,
            std::bind(&Renderer::cameraInfoCallback, this, std::placeholders::_1)
    );

    //image_transport::ImageTransport it_(shared_from_this());
    //this->image_sub_ = it_.subscribe<Renderer>("image", 1, &Renderer::imageCallback, this);
    this->image_sub_ = image_transport::create_subscription(
            this,
            "image",
            std::bind(&Renderer::imageCallback, this, std::placeholders::_1), 
            "raw");
    this->image_pub_ = image_transport::create_publisher(this, "dvs_rendering");
    this->undistorted_image_pub_ = image_transport::create_publisher(this, "dvs_undistorted");

    this->image_events_pub_ = image_transport::create_publisher(this, "dvs_accumulated_events");
    this->image_pub_events_edges_ = image_transport::create_publisher(this, "dvs_accumulated_events_edges");

    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 2; ++k)
            event_stats_[i].events_counter_[k] = 0;
    event_stats_[0].dt = 1;
    event_stats_[0].events_mean_lasttime_ = 0;
    event_stats_[0].events_mean_[0] = this->create_publisher<std_msgs::msg::Float32>("events_on_mean_1", 1);
    event_stats_[0].events_mean_[1] = this->create_publisher<std_msgs::msg::Float32>("events_off_mean_1", 1);
    event_stats_[1].dt = 5;
    event_stats_[1].events_mean_lasttime_ = 0;
    event_stats_[1].events_mean_[0] = this->create_publisher<std_msgs::msg::Float32>("events_on_mean_5", 1);
    event_stats_[1].events_mean_[1] = this->create_publisher<std_msgs::msg::Float32>("events_off_mean_5", 1);

} // Renderer::Renderer()

Renderer::~Renderer(){
    image_pub_.shutdown();
    undistorted_image_pub_.shutdown();
} // destructor

void Renderer::cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    got_camera_info_ = true;

    this->camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            camera_matrix_.at<double>(cv::Point(i, j)) = msg->k[i+j*3];

    this->dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
    for (int i = 0; i < msg->d.size(); i++)
        dist_coeffs_.at<double>(i) = msg->d[i]; 

} // cameraInfoCallback

void Renderer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{   
    sensor_msgs::msg::Image::SharedPtr image_events, image_events_edges;

    if (image_tracking_.imageCallback(msg, image_events, image_events_edges, this->now().seconds())){
        this->image_events_pub_.publish(image_events);
        this->image_pub_events_edges_.publish(image_events_edges);    
    }


    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
        return;
    }

    // convert to BGR image
    if (msg->encoding == "rgb8") cv::cvtColor(cv_ptr->image, last_image_, CV_RGB2BGR);
    if (msg->encoding == "mono8") {
        if (color_image_){
            cv::cvtColor(cv_ptr->image, last_image_, CV_BayerBG2BGR);
        } else {
            cv::cvtColor(cv_ptr->image, last_image_, CV_GRAY2BGR);
        }
    }

    if (!used_last_image_) {
        cv_bridge::CvImage cv_image;
        last_image_.copyTo(cv_image.image);
        cv_image.encoding = "bgr8";
        RCLCPP_INFO(this->get_logger(),"publish image from callback");
        image_pub_.publish(cv_image.toImageMsg());
    }
    used_last_image_ = false;

} // imageCallback

void Renderer::eventsCallback(
        const dvxplorer_interfaces::msg::EventArray::SharedPtr msg)
{
    for (int i = 0; i < msg->events.size(); ++i) {
        ++event_stats_[0].events_counter_[msg->events[i].polarity];
        ++event_stats_[1].events_counter_[msg->events[i].polarity];
    }

    publishStats();
    if (this->image_events_pub_.getNumSubscribers() == 0 && image_pub_events_edges_.getNumSubscribers() == 0)
        image_tracking_.eventsCallback(msg);

    // only create image if at least one subscriber
    if (image_pub_.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage cv_image;
        if (msg->events.size() > 0)
        {
            cv_image.header.stamp = msg->events[msg->events.size()/2].ts;
        }

        if (display_method_ == RED_BLUE)
        {
            cv_image.encoding = "bgr8";

            if (last_image_.rows == msg->height && last_image_.cols == msg->width)
            {
                last_image_.copyTo(cv_image.image);
                used_last_image_ = true;
            }
            else
            {
                cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
                cv_image.image = cv::Scalar(0,0,0);
            }

            for (int i = 0; i < msg->events.size(); ++i)
            {
                const int x = msg->events[i].x;
                const int y = msg->events[i].y;

                cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
                    msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
            }
        }
        else
        {
            cv_image.encoding = "mono8";
            if (last_image_.rows == msg->height && last_image_.cols == msg->width)
            {
                cv::cvtColor(last_image_, cv_image.image, CV_BGR2GRAY);
                used_last_image_ = true;
            }
            else
            {
                cv_image.image = cv::Mat(msg->height, msg->width, CV_8U);
                cv_image.image = cv::Scalar(128);
            }

            cv::Mat on_events = cv::Mat(msg->height, msg->width, CV_8U);
            on_events = cv::Scalar(0);

            cv::Mat off_events = cv::Mat(msg->height, msg->width, CV_8U);
            off_events = cv::Scalar(0);

            // count events per pixels with polarity
            for (int i = 0; i < msg->events.size(); ++i)
            {
                const int x = msg->events[i].x;
                const int y = msg->events[i].y;

                if (msg->events[i].polarity == 1)
                    on_events.at<uint8_t>(cv::Point(x, y))++;
                else
                    off_events.at<uint8_t>(cv::Point(x, y))++;
            }

            // scale image
            cv::normalize(on_events, on_events, 0, 128, cv::NORM_MINMAX, CV_8UC1);
            cv::normalize(off_events, off_events, 0, 127, cv::NORM_MINMAX, CV_8UC1);

            cv_image.image += on_events;
            cv_image.image -= off_events;
        }

        image_pub_.publish(cv_image.toImageMsg());

        if (got_camera_info_ && undistorted_image_pub_.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage cv_image2;
            cv_image2.encoding = cv_image.encoding;
            cv::undistort(cv_image.image, cv_image2.image, camera_matrix_, dist_coeffs_);
            undistorted_image_pub_.publish(cv_image2.toImageMsg());
        }
    }

} // eventsCallback

void Renderer::publishStats()
{
    std_msgs::msg::Float32 msg;
    rclcpp::Time now = this->now();

    for (int i = 0; i < 2; ++i)
    {
        if (event_stats_[i].events_mean_lasttime_ + event_stats_[i].dt <= now.seconds()) 
        {
            event_stats_[i].events_mean_lasttime_ = now.seconds();
            for (int k = 0; k < 2; ++k)
            {
                msg.data = (float)event_stats_[i].events_counter_[k] / event_stats_[i].dt;
                event_stats_[i].events_mean_[k]->publish(msg);
                event_stats_[i].events_counter_[k] = 0;
            }
        }
    }
} // publishStats
 
} // namespace dvs_renderer