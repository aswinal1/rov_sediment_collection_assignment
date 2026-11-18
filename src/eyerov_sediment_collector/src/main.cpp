#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "eyerov_sediment_collector/sediment_collector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<eyerov::SedimentCollectorNode>();

  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}