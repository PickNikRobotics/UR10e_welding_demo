#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

const rclcpp::Logger LOGGER = rclcpp::get_logger("plugin_task_description");

class PluginTaskDescriptionTestNode
{
public:
  PluginTaskDescriptionTestNode(const rclcpp::Node::SharedPtr& nh)
  {
    nh_ = nh;
  }

  void run()
  {
    // TODO Add hardcoded test case
    // - call service and load exemplary task description
    // - load workpiece to planning scene and publish workpiece tf
  }

private:
  rclcpp::Node::SharedPtr nh_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("plugin_task_description_test_node");

  PluginTaskDescriptionTestNode plugin_task_description_test_node(node);
  rclcpp::spin(node);
  return 0;
}
