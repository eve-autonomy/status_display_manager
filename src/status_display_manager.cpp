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

#include "status_display_manager/status_display_manager.hpp"

#include <fstream>

namespace status_display_manager
{

StatusDisplayManager::StatusDisplayManager(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("status_display_manager", options),
  display_dout_values_{
    // Listed in the order of DisplayStatus
    {false, false, false},  // DisplayStatus::STOP
    {true, false, false},   // DisplayStatus::RUNNING
    {false, true, false},   // DisplayStatus::OBSTACLE_DETECTION
    {false, false, true},   // DisplayStatus::TURN_RIGHT
    {true, false, true},    // DisplayStatus::TURN_LEFT
    {false, true, true},    // DisplayStatus::ERROR_DETECTION
    {true, true, false},    // DisplayStatus::PRESS_EMERGENCY_STOP_SWITCH
    {true, true, true}},    // DisplayStatus::HIDDEN
  sleep_time_at_running_state_(declare_parameter<int64_t>("sleep_time_at_running_state", 500)),
  display_dout_ports_()
{
  // read parameter
  this->declare_parameter("dout_ports_array", std::vector<int64_t>{});
  auto tmp_display_dout_port_list = this->get_parameter("dout_ports_array").as_integer_array();

  before_autoware_state_ = nullptr;
  status_display_state_ = DisplayStatus::HIDDEN;
  before_status_display_state_ = status_display_state_;
  emergency_switch_status_ = false;
  vehicle_turn_status_ = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;

  if (tmp_display_dout_port_list.size() > 0) {
    for (auto tmp_display_dout_port : tmp_display_dout_port_list) {
      display_dout_ports_.push_back(static_cast<uint32_t>(tmp_display_dout_port));
    }

    callback_group_subscribers_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subscriber_option = rclcpp::SubscriptionOptions();
    subscriber_option.callback_group = callback_group_subscribers_;

    callback_group_timer_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    sub_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateMachine>(
      "autoware_state_machine/state", rclcpp::QoS{3}.transient_local(),
      std::bind(&StatusDisplayManager::callbackStateMessage, this, std::placeholders::_1),
      subscriber_option);
    sub_emergency_stop_status_ =
      this->create_subscription<tier4_external_api_msgs::msg::HazardStatusStamped>(
      "hazard_status", rclcpp::QoS{1},
      std::bind(&StatusDisplayManager::callbackDiagStateMessage, this, std::placeholders::_1),
      subscriber_option);
    sub_turn_indicator_status_ =
      this->create_subscription<autoware_adapi_v1_msgs::msg::VehicleStatus>(
      "vehicle_status", rclcpp::SensorDataQoS(),
      std::bind(&StatusDisplayManager::callbackVehicleTurnMessage, this, std::placeholders::_1),
      subscriber_option);

    pub_dout_array_ =
      this->create_publisher<dio_ros_driver::msg::DIOArray>("/dio/dout_array", rclcpp::QoS(1));
    status_display_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&StatusDisplayManager::update, this),
      callback_group_timer_);
  }
}

StatusDisplayManager::~StatusDisplayManager()
{
  // Do not publish state when finished
  status_display_state_ = DisplayStatus::HIDDEN;
  before_status_display_state_ = status_display_state_;
}

void StatusDisplayManager::callbackStateMessage(
  const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr & msg)
{
  {
    std::unique_lock<std::mutex> lock(autoware_state_mutex_);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000.0,
      "[StatusDisplayManager::callbackStateMessage]"
      "service_layer_state: %u, control_layer_state: %u",
      msg->service_layer_state, msg->control_layer_state);

    queue_autoware_state_.push(msg);
  }
}

void StatusDisplayManager::callbackDiagStateMessage(
  const tier4_external_api_msgs::msg::HazardStatusStamped::ConstSharedPtr & msg)
{
  std::unique_lock<std::mutex> lock(emergency_switch_mutex_);
  
  // Search for /vehicle/007-obstacle_crash in diag_latent_fault or diag_single_point_fault
  bool found_obstacle_crash = false;
  
  // Search in diag_latent_fault
  for (const auto & diag : msg->status.diag_latent_fault) {
    if (diag.name == "/vehicle/007-obstacle_crash" && (diag.level == 2 || diag.level == 3)) {
      found_obstacle_crash = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000.0,
        "[StatusDisplayManager::callbackDiagStateMessage]"
        "name: %s, level: %u (latent_fault)",
        diag.name.c_str(), diag.level);
      break;
    }
  }
  
  // Search in diag_single_point_fault if not found in diag_latent_fault
  if (!found_obstacle_crash) {
    for (const auto & diag : msg->status.diag_single_point_fault) {
      if (diag.name == "/vehicle/007-obstacle_crash" && (diag.level == 2 || diag.level == 3)) {
        found_obstacle_crash = true;
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000.0,
          "[StatusDisplayManager::callbackDiagStateMessage]"
          "name: %s, level: %u (single_point_fault)",
          diag.name.c_str(), diag.level);
        break;
      }
    }
  }
  
  emergency_switch_status_ = found_obstacle_crash;
}

void StatusDisplayManager::callbackVehicleTurnMessage(
  const autoware_adapi_v1_msgs::msg::VehicleStatus::ConstSharedPtr & msg)
{
  std::unique_lock<std::mutex> lock(indicators_mutex_);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000.0,
    "[StatusDisplayManager::callbackVehicleTurnMessage]"
    "turn_indicators_status: %u",
    msg->turn_indicators.status);
  
  uint8_t turn_signal_value = msg->turn_indicators.status;
  if (turn_signal_value == 2) {  // LEFT
    vehicle_turn_status_ = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
  } else if (turn_signal_value == 3) {  // RIGHT
    vehicle_turn_status_ = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
  } else {  // UNKNOWN(0), DISABLE(1)
    vehicle_turn_status_ = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
  }
}

void StatusDisplayManager::controlStatusDisplay(builtin_interfaces::msg::Time time_stamp)
{
  dio_ros_driver::msg::DIOArray dout_msg;
  dio_ros_driver::msg::DIOPortValue dio_port_value;

  dout_msg.set__stamp(time_stamp);
  for (int i = 0; i < DISPLAY_DOUT_PORTS_NUM; i++) {
    dio_port_value.port = display_dout_ports_[i];
    dio_port_value.value = display_dout_values_[(uint8_t)status_display_state_].values_[i];
    dout_msg.values.push_back(dio_port_value);
  }
  pub_dout_array_->publish(dout_msg);
}

void StatusDisplayManager::statusDisplayManager(
  autoware_state_machine_msgs::msg::StateMachine autoware_state)
{
  if (
    autoware_state.control_layer_state == autoware_state_machine_msgs::msg::StateMachine::MANUAL) {
    // Manual
    status_display_state_ = DisplayStatus::HIDDEN;
  } else {
    // Auto
    if (
      status_display_state_ == DisplayStatus::PRESS_EMERGENCY_STOP_SWITCH ||
      status_display_state_ == DisplayStatus::ERROR_DETECTION) {
      // Nothing
    } else {
      switch (autoware_state.service_layer_state) {
        case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE:
        case autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL:
        case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION:
        case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION:
          status_display_state_ = DisplayStatus::STOP;
          break;
        case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE:
        case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART:
        case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING:
          status_display_state_ = DisplayStatus::RUNNING;
          ApplyTurnIndicatorsReport();
          if (status_display_state_ == DisplayStatus::RUNNING) {
            rclcpp::sleep_for(std::chrono::milliseconds(sleep_time_at_running_state_));
            ApplyEmergencyStopStatus();
          }
          break;
        case autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_RIGHT:
          status_display_state_ = DisplayStatus::TURN_RIGHT;
          break;
        case autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_LEFT:
          status_display_state_ = DisplayStatus::TURN_LEFT;
          break;
        case autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE:
          status_display_state_ = DisplayStatus::OBSTACLE_DETECTION;
          break;
        case autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP:
          status_display_state_ = DisplayStatus::ERROR_DETECTION;
          ApplyEmergencyStopStatus();
          break;
        case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_STOP_LINE:
        case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_OBSTACLE:
          status_display_state_ = DisplayStatus::RUNNING;
          ApplyTurnIndicatorsReport();
          break;
        default:
          // Nothing
          break;
      }
    }
  }

  if (before_status_display_state_ != status_display_state_) {
    before_status_display_state_ = status_display_state_;
    // publish
    controlStatusDisplay(autoware_state.stamp);
  }
}

void StatusDisplayManager::ApplyTurnIndicatorsReport()
{
  {
    std::unique_lock<std::mutex> lock(indicators_mutex_);
    if (vehicle_turn_status_ == autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT) {
      status_display_state_ = DisplayStatus::TURN_RIGHT;
    } else if (
      vehicle_turn_status_ == autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT) {
      status_display_state_ = DisplayStatus::TURN_LEFT;
    } else {
      // Nothing
    }
  }
}

void StatusDisplayManager::ApplyEmergencyStopStatus()
{
  {
    std::unique_lock<std::mutex> lock(emergency_switch_mutex_);
    if (emergency_switch_status_ == true) {
      status_display_state_ = DisplayStatus::PRESS_EMERGENCY_STOP_SWITCH;
    } else {
      // Nothing
    }
  }
}

void StatusDisplayManager::update()
{
  autoware_state_machine_msgs::msg::StateMachine autoware_state;
  {
    std::unique_lock<std::mutex> lock(autoware_state_mutex_);
    if (queue_autoware_state_.empty()) {
      if (before_autoware_state_ == nullptr) {
        return;
      }
      autoware_state = *before_autoware_state_;
    } else {
      autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr front_element =
        queue_autoware_state_.front();
      autoware_state = *front_element;
      queue_autoware_state_.pop();
      before_autoware_state_ = front_element;
    }
  }
  statusDisplayManager(autoware_state);
}

}  // namespace status_display_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(status_display_manager::StatusDisplayManager)
