// Copyright 2024 eve autonomy inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#ifndef STATUS_DISPLAY_MANAGER__STATUS_DISPLAY_MANAGER_HPP_
#define STATUS_DISPLAY_MANAGER__STATUS_DISPLAY_MANAGER_HPP_

#include <queue>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/subscription_options.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"
#include "dio_ros_driver/msg/dio_array.hpp"
#include "dio_ros_driver/msg/dio_port_value.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace status_display_manager {

class StatusDisplayManager : public rclcpp::Node {
public:
  explicit StatusDisplayManager(const rclcpp::NodeOptions & options);
  ~StatusDisplayManager();

private:
  #define DISPLAY_DOUT_PORTS_NUM (3)
  enum class DisplayStatus : int8_t {
    STOP = 0,
    RUNNING,
    OBSTACLE_DETECTION,
    TURN_RIGHT,
    TURN_LEFT,
    ERROR_DETECTION,
    PRESS_EMERGENCY_STOP_SWITCH,
    HIDDEN,
    DISPLAY_STATUS_NUMS
  };

  struct display_dout_values {
    bool values_[DISPLAY_DOUT_PORTS_NUM];
  } const display_dout_values_[(uint8_t)DisplayStatus::DISPLAY_STATUS_NUMS];

  // Callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_timer_;

  // Publisher
  rclcpp::Publisher<dio_ros_driver::msg::DIOArray>::SharedPtr pub_dout_array_;

  // Subscriber
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateMachine>::SharedPtr sub_state_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_dio_state_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    sub_turn_state_;

  // Timer callback
  rclcpp::TimerBase::SharedPtr status_display_update_timer_;

  std::queue<autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr> queue_autoware_state_;
  autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr before_autoware_state_;
  uint32_t sleep_time_at_running_state_;
  bool emergency_switch_status_;
  DisplayStatus before_status_display_state_;
  DisplayStatus status_display_state_;
  std::vector<uint32_t> display_dout_ports_;
  std::uint8_t vehicle_turn_status_;
  std::mutex autoware_state_mutex_;
  std::mutex indicators_mutex_;
  std::mutex emergency_switch_mutex_;

  void callbackStateMessage(
    const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr &msg);
  void callbackDiagStateMessage(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr &msg);
  void ApplyTurnIndicatorsReport();
  void ApplyEmergencyStopStatus();
  void callbackVehicleTurnMessage(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport &msg);
  void controlStatusDisplay(builtin_interfaces::msg::Time time_stamp);
  void statusDisplayManager(
    autoware_state_machine_msgs::msg::StateMachine autoware_state);
  void update();
};

}  // namespace status_display_manager
#endif  // STATUS_DISPLAY_MANAGER__STATUS_DISPLAY_MANAGER_HPP_
