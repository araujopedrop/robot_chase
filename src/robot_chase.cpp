#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

class RobotChase : public rclcpp::Node {
public:
  explicit RobotChase(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("Robot_Chase", node_options) {

    /*
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotChase::send_goal, this));
    */
  }

private:
  // rclcpp::TimerBase::SharedPtr timer_;

}; // class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  /*
  auto action_client = std::make_shared<RobotChase>();


  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }
    */

  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();

  return 0;
}