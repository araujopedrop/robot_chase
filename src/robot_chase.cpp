#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RobotChase : public rclcpp::Node {
public:
  explicit RobotChase(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("robot_chase_node", options) {

    using namespace std::placeholders;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->parent_frame_ = this->robot_catcher + "/base_link";
    this->child_frame_ = this->robot_escaper + "/base_link";

    // control loop of 100 Hz
    this->timer_check_bot_pos_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotChase::move_bot_to_coordinates, this));

    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        this->robot_catcher + "/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Robot_chase_node is up!");
  }

private:
  geometry_msgs::msg::Twist robot_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::TransformStamped desired_pos_;
  rclcpp::TimerBase::SharedPtr timer_check_bot_pos_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  const std::string robot_catcher = "rick";
  const std::string robot_escaper = "morty";

  std::string parent_frame_;
  std::string child_frame_;

  void get_desired_pos() {

    this->desired_pos_ =
        this->get_model_pose_from_tf(this->parent_frame_, this->child_frame_);
  }

  // Get transformation between parent_frame and child_frame
  geometry_msgs::msg::TransformStamped
  get_model_pose_from_tf(const std::string &parent_frame,
                         const std::string &child_frame) {

    geometry_msgs::msg::TransformStamped ts_;

    ts_.transform.translation.x = 0.0;
    ts_.transform.translation.y = 0.0;
    ts_.transform.translation.z = 0.0;
    ts_.transform.rotation.x = 0.0;
    ts_.transform.rotation.y = 0.0;
    ts_.transform.rotation.z = 0.0;
    ts_.transform.rotation.w = 0.0;

    // Look up for the transformation between parent_frame and child_frame
    try {
      ts_ = tf_buffer_->lookupTransform(parent_frame, child_frame,
                                        tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  parent_frame.c_str(), child_frame.c_str(), ex.what());
      return ts_;
    }

    return ts_;
  }

  void move_bot_to_coordinates() {

    float desired_x_ = 0.0;
    float desired_y_ = 0.0;

    float error_distance_ = 0.0;
    float error_angle_ = 0.0;

    float threshold_position = 0.5;
    float threshold_orientation = 0.02;

    float kp_yaw = 0.75;
    float kp_distance = 0.1;

    // Get running robot data
    this->get_desired_pos();

    // get data for control
    desired_x_ = this->desired_pos_.transform.translation.x;
    desired_y_ = this->desired_pos_.transform.translation.y;

    // calculate errors
    error_distance_ = sqrt(desired_x_ * desired_x_ + desired_y_ * desired_y_);
    error_angle_ = std::atan2(desired_y_, desired_x_);

    if (error_angle_ < -M_PI) {
      error_angle_ += 2 * M_PI;
    } else if (error_angle_ > M_PI) {
      error_angle_ -= 2 * M_PI;
    }

    if (abs(error_distance_) > threshold_position) {

      // If I reached position, start searching desired orientation
      // If not, continue searching position

      if (abs(error_angle_) < threshold_orientation) {

        this->robot_vel_.linear.x = kp_distance * abs(error_distance_);
        this->robot_vel_.angular.z = 0.0;
        publisher_->publish(this->robot_vel_);

      } else {
        this->robot_vel_.linear.x = kp_distance * abs(error_distance_);
        this->robot_vel_.angular.z = kp_yaw * error_angle_;
        publisher_->publish(this->robot_vel_);
      }

    } else {

      // Stop robot
      this->robot_vel_.linear.x = 0.0;
      this->robot_vel_.angular.z = 0.0;
      publisher_->publish(this->robot_vel_);
    }
  }

}; // class GoToPose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}