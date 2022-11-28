#include <rclcpp/rclcpp.hpp>
#include "dvxplorer_ros2/dvxplorer_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dvxplorer_ros2_driver::DvXplorer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}