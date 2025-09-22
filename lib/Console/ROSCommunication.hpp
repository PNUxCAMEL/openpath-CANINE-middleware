//
// Created by jh on 25. 2. 3.
//

#ifndef ROSCOMMUNICATION_HPP
#define ROSCOMMUNICATION_HPP

#include <Eigen/Dense>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <canine_msgs_v2/msg/canine_state.hpp>
#include <canine_msgs_v2/msg/canine_command.hpp>
#include "SharedMemory.h"

class ROSCommunication : public rclcpp::Node
{
public:
  ROSCommunication(const rclcpp::NodeOptions& opts);
  void publishStates();
  void package_canine_state_msg(canine_msgs_v2::msg::CANINEState& msg);
  void topic_callback_canine_command(const canine_msgs_v2::msg::CANINECommand::SharedPtr msg) const;

private:
  rclcpp::TimerBase::SharedPtr timer_canine_states;
  rclcpp::Publisher<canine_msgs_v2::msg::CANINEState>::SharedPtr publisher_canine_states;
  rclcpp::Subscription<canine_msgs_v2::msg::CANINECommand>::SharedPtr subscription_canine_command;


};



#endif //ROSCOMMUNICATION_HPP

