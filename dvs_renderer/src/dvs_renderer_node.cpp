#include <rclcpp/rclcpp.hpp>
#include "dvs_renderer/dvs_renderer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "TRACE 1" << std::endl;
  auto node = std::make_shared<dvs_renderer::Renderer>();
  std::cout << "TRACE 2" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}