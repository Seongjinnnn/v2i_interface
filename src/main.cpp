#include <rclcpp/rclcpp.hpp>
#include "v2i_interface/v2i_interface_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<v2i_interface::V2IInterfaceNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
