#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>

class Coordinates {
public:
  Coordinates(float x, float y, float z, float roll, float pitch, float yaw)
      : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}

private:
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};

class RobotChase : public rclcpp::Node {
public:
  explicit RobotChase(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("robot_chase_node", options) {

    using namespace std::placeholders;

    ptr_objCoordinates =
        std::make_shared<Coordinates>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->parent_frame_ = this->robot_catcher + "/base_link";
    this->child_frame_ = this->robot_escaper + "/base_link";

    // control loop of 100 Hz
    this->timer_check_bot_pos_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&RobotChase::move_bot, this));

    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        this->robot_catcher + "/cmd_vel", 10);

    this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->robot_catcher + "/odom", 10,
        std::bind(&RobotChase::odom_callback, this, _1));
  }

private:
  /*



  geometry_msgs::msg::Pose2D desired_pos_;
  
  geometry_msgs::msg::Twist robot_vel_;

  */
  
  geometry_msgs::msg::Pose2D current_pos_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::TransformStamped desired_pos_;
  rclcpp::TimerBase::SharedPtr timer_check_bot_pos_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<Coordinates> ptr_objCoordinates;

  const std::string robot_catcher = "rick";
  const std::string robot_escaper = "morty";

  std::string parent_frame_;
  std::string child_frame_;

  void move_bot() {

    if (ptr_objCoordinates) {
      this->move_bot_to_coordinates();
    } else {
    }
  }

  void move_bot_to_coordinates() {

    this->desired_pos_ =
        this->get_model_pose_from_tf(this->parent_frame_, this->child_frame_);

    this->desired_pos_.transform.translation.x;
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

    // Return desired pos

    return ts_;
  }

  /*

  void execute() {

    float threshold_ = 0.1;
    float current_angle_ = 0.0;
    float desi
    float current_y_ = 0.0;
    float desired_x_ = 0.0;
    float desired_y_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Executing goal");
red_angle_ = 0.0;
    float target_angle_ = 0.0;
    float error_angle_ = 0.0;
    float current_x_ = 0.0;
    // get data for control
    current_x_ = this->current_pos_.x;
    current_y_ = this->current_pos_.y;
    desired_x_ = this->desired_pos_.x;
    desired_y_ = this->desired_pos_.y;
    current_angle_ = this->current_pos_.theta;
    desired_angle_ = this->desired_pos_.theta *
                     0.01745; // Action input is given in degrees: *(2*PI / 360)

    // Show in terminal
    RCLCPP_INFO(this->get_logger(), "Current position: x: %f - y: %f",
                current_x_, current_y_);

    RCLCPP_INFO(this->get_logger(), "Desired position: x: %f - y: %f",
                desired_x_, desired_y_);

    RCLCPP_INFO(this->get_logger(), "Delta x: %f - Delta y: %f",
                std::abs(current_x_ - desired_x_),
                std::abs(current_y_ - desired_y_));

    // Control loop
    rclcpp::Rate loop_rate(10);
    while (this->goal_reached(current_x_, current_y_, current_angle_,
                              desired_x_, desired_y_, desired_angle_,
                              threshold_) == false) {

      // Check if goal is cancelled
      if (true) {

        // Stop robot
        this->robot_vel_.linear.x = 0.0;
        this->robot_vel_.angular.z = 0.0;
        publisher_->publish(this->robot_vel_);

        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // If I reached position, start searching desired orientation
      // If not, continue searching position

      if (this->pos_reached(current_x_, current_y_, desired_x_, desired_y_,
                            threshold_) == false) {

        target_angle_ =
            std::atan2(desired_y_ - current_y_, desired_x_ - current_x_);

        this->robot_vel_.linear.x = 0.2;

      } else {

        target_angle_ = desired_angle_;
        this->robot_vel_.linear.x = 0.0;
      }

      // Calculate error and publish velocities
      error_angle_ = target_angle_ - current_angle_;

      if (error_angle_ < -M_PI) {
        error_angle_ += 2 * M_PI;
      } else if (error_angle_ > M_PI) {
        error_angle_ -= 2 * M_PI;
      }

      this->robot_vel_.angular.z = error_angle_;
      publisher_->publish(this->robot_vel_);

      // Update data
      current_x_ = this->current_pos_.x;
      current_y_ = this->current_pos_.y;
      current_angle_ = this->current_pos_.theta;

      loop_rate.sleep();
    }

    // If goal is done
    if (rclcpp::ok()) {

      // Stop robot
      this->robot_vel_.linear.x = 0.0;
      this->robot_vel_.angular.z = 0.0;
      publisher_->publish(this->robot_vel_);

      // Send result
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  bool pos_reached(float current_x, float current_y, float desired_x,
                   float desired_y, float threshold) {

    if (std::abs(current_x - desired_x) < threshold) {
      if (std::abs(current_y - desired_y) < threshold) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return true;
      }
    }

    return false;
  }



  bool goal_reached(float current_x, float current_y, float current_angle,
                    float desired_x, float desired_y, float desired_angle,
                    float threshold) {

    if (pos_reached(current_x, current_y, desired_x, desired_y, threshold) ==
        true) {
      if (std::abs(current_angle - desired_angle) < threshold) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return true;
      }
    }

    return false;
  }

*/
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    geometry_msgs::msg::Quaternion quaternion_;

    double roll, pitch, yaw;

    float x_;
    float y_;

    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    // Quaterion 2 Euler


    quaternion_ = msg->pose.pose.orientation;

    //tf2::Quaternion tf_quaternion;
    //tf2::fromMsg(quaternion_, tf_quaternion);
    //tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);


    this->current_pos_.x = x_;
    this->current_pos_.y = y_;
    this->current_pos_.theta = yaw;


  }

  



}; // class GoToPose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}