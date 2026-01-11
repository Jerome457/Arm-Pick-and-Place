#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("Pick_and_Place server");

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("Pick_and_Place", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to pick.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}