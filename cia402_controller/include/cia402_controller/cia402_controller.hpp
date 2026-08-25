// Copyright 2026 VHIT
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
// limitations under the License.

#ifndef CIA402_CONTROLLER__CIA402_CONTROLLER_HPP_
#define CIA402_CONTROLLER__CIA402_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cia402_controller/cia402_state_machine.hpp"

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <cia402_interfaces/action/execute_drive_command.hpp>
#include <cia402_interfaces/msg/drive_state.hpp>
#include <cia402_interfaces/msg/drive_state_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_server_goal_handle.hpp>

namespace cia402_controller
{

struct DriveContext
{
  std::string joint_name;

  uint16_t status_word{0};
  uint16_t control_word{CONTROLWORD_DISABLE_VOLTAGE};
  int8_t modes_of_operation_display{0};
  int8_t modes_of_operation{0};

  DriveState current_state{DriveState::UNKNOWN};
  DriveState previous_state{DriveState::UNKNOWN};
  DriveState expected_state{DriveState::UNKNOWN};

  bool feedback_valid{false};
  bool transition_complete{false};
  bool fault_reset_asserted{false};
  bool waiting_for_mode{false};
  int64_t step_elapsed_ns{0};
};

struct DriveSnapshot
{
  builtin_interfaces::msg::Time stamp;
  std::string joint_name;
  uint16_t status_word{0};
  uint16_t control_word{0};
  int8_t modes_of_operation_display{0};
  int8_t modes_of_operation{0};
  DriveState state{DriveState::UNKNOWN};
  bool feedback_valid{false};
};

class Cia402Controller : public controller_interface::ControllerInterface
{
public:
  Cia402Controller();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  using ExecuteDriveCommand = cia402_interfaces::action::ExecuteDriveCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteDriveCommand>;
  using ActionServer = rclcpp_action::Server<ExecuteDriveCommand>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ExecuteDriveCommand>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  struct ActiveGoal
  {
    static constexpr uint8_t STATE_ACTIVE = 0;
    static constexpr uint8_t STATE_CANCEL_REQUESTED = 1;
    static constexpr uint8_t STATE_TERMINATING = 2;
    static constexpr uint8_t STATE_FINISHED = 3;

    RealtimeGoalHandlePtr handle;
    uint64_t activation_generation{0};
    DriveCommand command{DriveCommand::DISABLE_VOLTAGE};
    std::vector<std::size_t> selected_indices;
    int64_t timeout_ns{0};

    bool started{false};

    std::atomic<uint8_t> execution_state{STATE_ACTIVE};
    std::atomic<bool> reservation_released{false};
    std::atomic<int64_t> elapsed_ns{0};
    std::atomic<uint8_t> phase{ExecuteDriveCommand::Feedback::PHASE_ACCEPTED};
  };

  using ActiveGoalPtr = std::shared_ptr<ActiveGoal>;

  std::vector<std::string> joint_names_;
  std::vector<DriveContext> drive_contexts_;

  double default_timeout_seconds_{5.0};
  double step_timeout_seconds_{1.0};
  double feedback_rate_{20.0};
  int8_t default_mode_of_operation_{8};
  DriveCommand fallback_command_{DriveCommand::QUICK_STOP};
  int64_t default_timeout_ns_{5000000000LL};
  int64_t step_timeout_ns_{1000000000LL};
  int64_t feedback_period_ns_{50000000LL};
  int64_t state_publish_elapsed_ns_{0};
  bool state_published_{false};
  rclcpp::Duration feedback_period_{0, 0};

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  control_word_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  modes_of_operation_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  status_word_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  modes_of_operation_display_interfaces_;

  rclcpp::Publisher<cia402_interfaces::msg::DriveStateArray>::SharedPtr state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<cia402_interfaces::msg::DriveStateArray>>
  realtime_state_publisher_;

  ActionServer::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr goal_monitor_timer_;
  realtime_tools::RealtimeBuffer<ActiveGoalPtr> active_goal_buffer_;
  std::atomic<bool> accepting_goals_{false};
  std::atomic<uint64_t> activation_generation_{0};
  std::atomic<uint64_t> reserved_generation_{0};
  std::mutex action_callback_mutex_;

  std::mutex snapshot_mutex_;
  std::vector<DriveSnapshot> non_realtime_snapshots_;

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDriveCommand::Goal> goal,
    uint64_t activation_generation);

  rclcpp_action::CancelResponse cancel_callback(
    const std::shared_ptr<GoalHandle> goal_handle,
    uint64_t activation_generation);

  void accepted_callback(
    std::shared_ptr<GoalHandle> goal_handle,
    uint64_t activation_generation);

  void monitor_goal(const ActiveGoalPtr & active_goal);

  bool bind_ordered_interfaces();

  bool read_drive_feedback(const rclcpp::Time & time, bool log_errors);

  bool advance_drive(
    DriveContext & context,
    std::size_t drive_index,
    DriveCommand command,
    int64_t period_ns,
    uint8_t & failure_code,
    std::string & failure_message);

  void stage_fallback_commands(const std::vector<std::size_t> & selected_indices);

  void finish_goal(
    const ActiveGoalPtr & active_goal,
    bool success,
    uint8_t result_code,
    const std::string & message,
    const rclcpp::Time & time,
    bool canceled = false);

  void abort_active_goal_on_deactivate();

  void publish_drive_states(const rclcpp::Time & time, int64_t period_ns);

  void clear_bound_interfaces();

  void release_goal_reservation(uint64_t activation_generation);

  void update_non_realtime_snapshots(const rclcpp::Time & time);

  static cia402_interfaces::msg::DriveState make_drive_state_message(
    const DriveSnapshot & snapshot);

  DriveSnapshot make_drive_snapshot(std::size_t drive_index, const rclcpp::Time & time) const;
};

}  // namespace cia402_controller

#endif  // CIA402_CONTROLLER__CIA402_CONTROLLER_HPP_
