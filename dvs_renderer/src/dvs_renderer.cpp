
#include "dvs_renderer/dvs_renderer.hpp"

namespace dvs_renderer{

Renderer::Renderer(rclcpp::Node::SharedPtr node)
: node_(node), image_tracking_(node)
{   
    this->got_camera_info_ = false;

    // get parameters of display method
    std::string display_method_str;
    this->node_->declare_parameter("display_method", "");
    this->node_->get_parameter("display_method", display_method_str);
    this->display_method_ = (display_method_str == std::string("grayscale")) ? GRAYSCALE : RED_BLUE;
    
    this->node_->declare_parameter("color", false);
    this->node_->get_parameter("color", this->color_image_);
    this->used_last_image_ = false;

    // Subscribers and publishers
    this->event_sub_ = this->node_->create_subscription<dvxplorer_interfaces::msg::EventArray>(
            "events",
            1,
            std::bind(&Renderer::eventsCallback, this, std::placeholders::_1)
    );

} // Renderer::Renderer()

} // namespace dvs_renderer