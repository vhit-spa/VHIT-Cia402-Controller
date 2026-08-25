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

#include "cia402_controller/cia402_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <controller_interface/helpers.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace cia402_controller
{
namespace
{

constexpr double VALUE_EPSILON = 1e-6;
constexpr int64_t NANOSECONDS_PER_SECOND = 1000000000LL;
constexpr double MAX_DURATION_SECONDS =
  static_cast<double>(std::numeric_limits<int32_t>::max());
constexpr double MIN_FEEDBACK_RATE_HZ = 1.0;
constexpr double MAX_FEEDBACK_RATE_HZ = 1000.0;

bool read_uint16_interface_value(const double value, uint16_t & output)
{
  if (!std::isfinite(value)) {
    return false;
  }
  const double rounded = std::round(value);
  if (std::fabs(value - rounded) > VALUE_EPSILON || rounded < 0.0 ||
    rounded > static_cast<double>(std::numeric_limits<uint16_t>::max()))
  {
    return false;
  }
  output = static_cast<uint16_t>(rounded);
  return true;
}

bool read_int8_interface_value(const double value, int8_t & output)
{
  if (!std::isfinite(value)) {
    return false;
  }
  const double rounded = std::round(value);
  if (std::fabs(value - rounded) > VALUE_EPSILON ||
    rounded < static_cast<double>(std::numeric_limits<int8_t>::min()) ||
    rounded > static_cast<double>(std::numeric_limits<int8_t>::max()))
  {
    return false;
  }
  output = static_cast<int8_t>(rounded);
  return true;
}

bool valid_goal_timeout(const builtin_interfaces::msg::Duration & timeout)
{
  return timeout.sec >= 0 && timeout.nanosec < static_cast<uint32_t>(NANOSECONDS_PER_SECOND);
}

int64_t timeout_to_nanoseconds(
  const builtin_interfaces::msg::Duration & timeout,
  const int64_t default_timeout_ns)
{
  if (timeout.sec == 0 && timeout.nanosec == 0U) {
    return default_timeout_ns;
  }
  return static_cast<int64_t>(timeout.sec) * NANOSECONDS_PER_SECOND + timeout.nanosec;
}

bool duration_seconds_are_representable(const double seconds)
{
  return std::isfinite(seconds) && seconds > 0.0 && seconds <= MAX_DURATION_SECONDS;
}

int64_t seconds_to_nanoseconds(const double seconds)
{
  return static_cast<int64_t>(std::llround(seconds * NANOSECONDS_PER_SECOND));
}

int64_t saturating_add(const int64_t left, const int64_t right)
{
  if (right <= 0) {
    return left;
  }
  if (left > std::numeric_limits<int64_t>::max() - right) {
    return std::numeric_limits<int64_t>::max();
  }
  return left + right;
}

uint16_t hold_control_word_for_state(const DriveState state)
{
  switch (state) {
    case DriveState::READY_TO_SWITCH_ON:
      return CONTROLWORD_SHUTDOWN;
    case DriveState::SWITCHED_ON:
      return CONTROLWORD_SWITCH_ON;
    case DriveState::OPERATION_ENABLED:
      return CONTROLWORD_ENABLE_OPERATION;
    case DriveState::QUICK_STOP_ACTIVE:
      return CONTROLWORD_QUICK_STOP;
    case DriveState::NOT_READY_TO_SWITCH_ON:
    case DriveState::SWITCH_ON_DISABLED:
    case DriveState::FAULT_REACTION_ACTIVE:
    case DriveState::FAULT:
    case DriveState::UNKNOWN:
      return CONTROLWORD_DISABLE_VOLTAGE;
  }
  return CONTROLWORD_DISABLE_VOLTAGE;
}

builtin_interfaces::msg::Duration nanoseconds_to_duration_message(const int64_t nanoseconds)
{
  return static_cast<builtin_interfaces::msg::Duration>(
    rclcpp::Duration::from_nanoseconds(std::max<int64_t>(0, nanoseconds)));
}

}  // namespace

Cia402Controller::Cia402Controller()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn Cia402Controller::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<double>("default_timeout", 5.0);
    auto_declare<double>("step_timeout", 1.0);
    auto_declare<double>("feedback_rate", 20.0);
    auto_declare<int>("default_mode_of_operation", 8);
    auto_declare<int>(
      "fallback_command", static_cast<int>(DriveCommand::QUICK_STOP));
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to declare controller parameters: %s",
      exception.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Cia402Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(joint_names_.size() * 2U);
  for (const auto & joint_name : joint_names_) {
    configuration.names.push_back(joint_name + "/control_word");
    configuration.names.push_back(joint_name + "/modes_of_operation");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
Cia402Controller::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names.reserve(joint_names_.size() * 2U);
  for (const auto & joint_name : joint_names_) {
    configuration.names.push_back(joint_name + "/status_word");
    configuration.names.push_back(joint_name + "/modes_of_operation_display");
  }
  return configuration;
}

controller_interface::CallbackReturn Cia402Controller::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  int configured_mode = 0;
  int configured_fallback_command = 0;
  node->get_parameter("joints", joint_names_);
  node->get_parameter("default_timeout", default_timeout_seconds_);
  node->get_parameter("step_timeout", step_timeout_seconds_);
  node->get_parameter("feedback_rate", feedback_rate_);
  node->get_parameter("default_mode_of_operation", configured_mode);
  node->get_parameter("fallback_command", configured_fallback_command);

  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' must contain at least one joint");
    return CallbackReturn::ERROR;
  }

  std::unordered_set<std::string> unique_joint_names;
  for (const auto & joint_name : joint_names_) {
    if (joint_name.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' contains an empty joint name");
      return CallbackReturn::ERROR;
    }
    if (!unique_joint_names.insert(joint_name).second) {
      RCLCPP_ERROR(
        node->get_logger(), "Joint '%s' occurs more than once in parameter 'joints'",
        joint_name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  if (!duration_seconds_are_representable(default_timeout_seconds_)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'default_timeout' must be positive and representable as a ROS duration");
    return CallbackReturn::ERROR;
  }
  if (!duration_seconds_are_representable(step_timeout_seconds_)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'step_timeout' must be positive and representable as a ROS duration");
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(feedback_rate_) || feedback_rate_ < MIN_FEEDBACK_RATE_HZ ||
    feedback_rate_ > MAX_FEEDBACK_RATE_HZ)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'feedback_rate' must be finite and between 1 Hz and 1000 Hz");
    return CallbackReturn::ERROR;
  }
  if (configured_mode == 0 || configured_mode < std::numeric_limits<int8_t>::min() ||
    configured_mode > std::numeric_limits<int8_t>::max())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'default_mode_of_operation' must be a non-zero signed 8-bit value");
    return CallbackReturn::ERROR;
  }
  if (configured_fallback_command != static_cast<int>(DriveCommand::DISABLE_OPERATION) &&
    configured_fallback_command != static_cast<int>(DriveCommand::DISABLE_VOLTAGE) &&
    configured_fallback_command != static_cast<int>(DriveCommand::QUICK_STOP))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Parameter 'fallback_command' must be 4 (Disable operation), 5 (Disable voltage), "
      "or 6 (Quick stop)");
    return CallbackReturn::ERROR;
  }

  default_mode_of_operation_ = static_cast<int8_t>(configured_mode);
  fallback_command_ = static_cast<DriveCommand>(configured_fallback_command);
  default_timeout_ns_ = seconds_to_nanoseconds(default_timeout_seconds_);
  step_timeout_ns_ = seconds_to_nanoseconds(step_timeout_seconds_);
  feedback_period_ns_ = std::max<int64_t>(
    1, seconds_to_nanoseconds(1.0 / feedback_rate_));
  feedback_period_ = rclcpp::Duration::from_nanoseconds(feedback_period_ns_);
  state_publish_elapsed_ns_ = 0;
  state_published_ = false;

  drive_contexts_.clear();
  drive_contexts_.reserve(joint_names_.size());
  non_realtime_snapshots_.clear();
  non_realtime_snapshots_.resize(joint_names_.size());
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    DriveContext context;
    context.joint_name = joint_names_[index];
    context.modes_of_operation = default_mode_of_operation_;
    drive_contexts_.push_back(std::move(context));
    non_realtime_snapshots_[index].joint_name = joint_names_[index];
    RCLCPP_INFO(
      node->get_logger(), "Configured CiA 402 drive '%s'", joint_names_[index].c_str());
  }

  state_publisher_ = node->create_publisher<cia402_interfaces::msg::DriveStateArray>(
    "~/drive_states", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  realtime_state_publisher_ = std::make_shared<
    realtime_tools::RealtimePublisher<cia402_interfaces::msg::DriveStateArray>>(state_publisher_);
  realtime_state_publisher_->msg_.states.resize(joint_names_.size());
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    realtime_state_publisher_->msg_.states[index].joint_name = joint_names_[index];
  }

  action_server_.reset();
  goal_monitor_timer_.reset();
  active_goal_buffer_.writeFromNonRT(ActiveGoalPtr{});
  accepting_goals_.store(false);
  activation_generation_.fetch_add(1);
  reserved_generation_.store(0);

  return CallbackReturn::SUCCESS;
}

bool Cia402Controller::bind_ordered_interfaces()
{
  control_word_interfaces_.clear();
  modes_of_operation_command_interfaces_.clear();
  status_word_interfaces_.clear();
  modes_of_operation_display_interfaces_.clear();

  const bool control_words_found = controller_interface::get_ordered_interfaces(
    command_interfaces_, joint_names_, "control_word", control_word_interfaces_);
  const bool modes_found = controller_interface::get_ordered_interfaces(
    command_interfaces_, joint_names_, "modes_of_operation",
    modes_of_operation_command_interfaces_);
  const bool status_words_found = controller_interface::get_ordered_interfaces(
    state_interfaces_, joint_names_, "status_word", status_word_interfaces_);
  const bool mode_displays_found = controller_interface::get_ordered_interfaces(
    state_interfaces_, joint_names_, "modes_of_operation_display",
    modes_of_operation_display_interfaces_);

  if (!control_words_found || !modes_found || !status_words_found || !mode_displays_found) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Could not bind every control_word, modes_of_operation, status_word, and "
      "modes_of_operation_display interface");
    control_word_interfaces_.clear();
    modes_of_operation_command_interfaces_.clear();
    status_word_interfaces_.clear();
    modes_of_operation_display_interfaces_.clear();
    return false;
  }
  return true;
}

void Cia402Controller::clear_bound_interfaces()
{
  control_word_interfaces_.clear();
  modes_of_operation_command_interfaces_.clear();
  status_word_interfaces_.clear();
  modes_of_operation_display_interfaces_.clear();
  release_interfaces();
}

controller_interface::CallbackReturn Cia402Controller::on_activate(
  const rclcpp_lifecycle::State &)
{
  accepting_goals_.store(false);
  const uint64_t generation = activation_generation_.fetch_add(1) + 1;
  if (!bind_ordered_interfaces()) {
    return CallbackReturn::ERROR;
  }

  const rclcpp::Time now = get_node()->now();
  if (!read_drive_feedback(now, true)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Cannot activate with invalid CiA 402 feedback interfaces");
    clear_bound_interfaces();
    return CallbackReturn::ERROR;
  }

  // Preserve each observed CiA 402 state while taking ownership. In
  // particular, activation must not remove torque from an already enabled
  // drive merely because this controller was started.
  for (std::size_t index = 0; index < drive_contexts_.size(); ++index) {
    auto & context = drive_contexts_[index];
    context.control_word = hold_control_word_for_state(context.current_state);
    context.modes_of_operation = context.modes_of_operation_display;
    context.expected_state = context.current_state;
    context.transition_complete = false;
    context.fault_reset_asserted = false;
    context.waiting_for_mode = false;
    context.step_elapsed_ns = 0;
    control_word_interfaces_[index].get().set_value(context.control_word);
    modes_of_operation_command_interfaces_[index].get().set_value(
      context.modes_of_operation);
  }
  update_non_realtime_snapshots(now);
  active_goal_buffer_.writeFromNonRT(ActiveGoalPtr{});
  reserved_generation_.store(0);

  try {
    action_server_ = rclcpp_action::create_server<ExecuteDriveCommand>(
      get_node(), "~/execute_drive_command",
      [this, generation](
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<const ExecuteDriveCommand::Goal> goal)
      {
        return goal_callback(uuid, goal, generation);
      },
      [this, generation](const std::shared_ptr<GoalHandle> goal_handle)
      {
        return cancel_callback(goal_handle, generation);
      },
      [this, generation](const std::shared_ptr<GoalHandle> goal_handle)
      {
        accepted_callback(goal_handle, generation);
      });
    goal_monitor_timer_ = get_node()->create_wall_timer(
      feedback_period_.to_chrono<std::chrono::nanoseconds>(),
      [this]() {
        const ActiveGoalPtr active_goal = *active_goal_buffer_.readFromNonRT();
        monitor_goal(active_goal);
      });
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to create ExecuteDriveCommand action server: %s",
      exception.what());
    action_server_.reset();
    goal_monitor_timer_.reset();
    clear_bound_interfaces();
    return CallbackReturn::ERROR;
  }

  state_publish_elapsed_ns_ = 0;
  state_published_ = false;
  accepting_goals_.store(true);
  RCLCPP_INFO(
    get_node()->get_logger(), "CiA 402 command action and drive-state publisher are active");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Cia402Controller::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  accepting_goals_.store(false);
  activation_generation_.fetch_add(1);
  std::lock_guard<std::mutex> callback_lock(action_callback_mutex_);
  abort_active_goal_on_deactivate();

  std::vector<std::size_t> every_drive(drive_contexts_.size());
  for (std::size_t index = 0; index < every_drive.size(); ++index) {
    every_drive[index] = index;
  }
  stage_fallback_commands(every_drive);

  action_server_.reset();
  goal_monitor_timer_.reset();
  reserved_generation_.store(0);
  active_goal_buffer_.writeFromNonRT(ActiveGoalPtr{});

  clear_bound_interfaces();
  return CallbackReturn::SUCCESS;
}

bool Cia402Controller::read_drive_feedback(const rclcpp::Time &, const bool log_errors)
{
  bool all_valid = true;
  for (std::size_t index = 0; index < drive_contexts_.size(); ++index) {
    auto & context = drive_contexts_[index];
    uint16_t status_word = 0;
    int8_t mode_display = 0;
    const bool status_valid = read_uint16_interface_value(
      status_word_interfaces_[index].get().get_value(), status_word);
    const bool mode_valid = read_int8_interface_value(
      modes_of_operation_display_interfaces_[index].get().get_value(), mode_display);

    context.feedback_valid = status_valid && mode_valid;
    if (!context.feedback_valid) {
      context.previous_state = context.current_state;
      context.current_state = DriveState::UNKNOWN;
      all_valid = false;
      if (log_errors) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Drive '%s' returned non-finite, fractional, or out-of-range CiA 402 feedback",
          context.joint_name.c_str());
      }
      continue;
    }

    context.status_word = status_word;
    context.modes_of_operation_display = mode_display;
    context.previous_state = context.current_state;
    context.current_state = decode_status_word(status_word);
  }
  return all_valid;
}

bool Cia402Controller::advance_drive(
  DriveContext & context,
  const std::size_t drive_index,
  const DriveCommand command,
  const int64_t period_ns,
  uint8_t & failure_code,
  std::string & failure_message)
{
  if (!context.feedback_valid || context.current_state == DriveState::UNKNOWN) {
    failure_code = ExecuteDriveCommand::Result::RESULT_COMMUNICATION_ERROR;
    failure_message = "Invalid feedback from drive '" + context.joint_name + "'";
    return false;
  }

  if ((context.current_state == DriveState::FAULT ||
    context.current_state == DriveState::FAULT_REACTION_ACTIVE) &&
    command != DriveCommand::FAULT_RESET)
  {
    failure_code = ExecuteDriveCommand::Result::RESULT_DRIVE_FAULT;
    failure_message = "Drive '" + context.joint_name + "' entered " +
      drive_state_name(context.current_state);
    return false;
  }

  if (command == DriveCommand::ENABLE_OPERATION) {
    context.modes_of_operation = default_mode_of_operation_;
    modes_of_operation_command_interfaces_[drive_index].get().set_value(
      default_mode_of_operation_);
  }

  // Fault reset is a one-update-cycle pulse. Clearing bit 7 after asserting it
  // guarantees that a future fault reset can produce another rising edge.
  if (command == DriveCommand::FAULT_RESET && context.current_state == DriveState::FAULT) {
    if (!context.fault_reset_asserted) {
      context.fault_reset_asserted = true;
      context.step_elapsed_ns = 0;
      context.expected_state = DriveState::SWITCH_ON_DISABLED;
      context.control_word = CONTROLWORD_FAULT_RESET;
    } else {
      context.control_word = CONTROLWORD_DISABLE_VOLTAGE;
      context.step_elapsed_ns = saturating_add(context.step_elapsed_ns, period_ns);
      if (context.step_elapsed_ns > step_timeout_ns_) {
        failure_code = ExecuteDriveCommand::Result::RESULT_TIMEOUT;
        failure_message = "Fault reset timed out for drive '" + context.joint_name + "'";
        return false;
      }
    }
    control_word_interfaces_[drive_index].get().set_value(context.control_word);
    context.transition_complete = false;
    return true;
  }

  const bool at_final_enable_step = command == DriveCommand::ENABLE_OPERATION &&
    (context.current_state == DriveState::SWITCHED_ON ||
    context.current_state == DriveState::QUICK_STOP_ACTIVE ||
    context.current_state == DriveState::OPERATION_ENABLED);
  if (at_final_enable_step &&
    context.modes_of_operation_display != default_mode_of_operation_)
  {
    if (!context.waiting_for_mode) {
      context.waiting_for_mode = true;
      context.step_elapsed_ns = 0;
    } else {
      context.step_elapsed_ns = saturating_add(context.step_elapsed_ns, period_ns);
    }
    if (context.step_elapsed_ns > step_timeout_ns_) {
      failure_code = ExecuteDriveCommand::Result::RESULT_TIMEOUT;
      failure_message = "Mode of operation confirmation timed out for drive '" +
        context.joint_name + "'";
      return false;
    }

    if (context.current_state == DriveState::QUICK_STOP_ACTIVE) {
      context.control_word = CONTROLWORD_QUICK_STOP;
    } else {
      context.control_word = CONTROLWORD_SWITCH_ON;
    }
    control_word_interfaces_[drive_index].get().set_value(context.control_word);
    context.transition_complete = false;
    return true;
  }
  context.waiting_for_mode = false;

  const TransitionStep step = calculate_transition_step(context.current_state, command);
  if (!step.valid) {
    failure_code = ExecuteDriveCommand::Result::RESULT_INVALID_TRANSITION;
    failure_message = "Command '" + std::string(drive_command_name(command)) +
      "' is invalid from state '" + drive_state_name(context.current_state) +
      "' for drive '" + context.joint_name + "'";
    return false;
  }

  context.control_word = step.control_word;
  control_word_interfaces_[drive_index].get().set_value(context.control_word);

  if (step.goal_reached) {
    context.transition_complete = true;
    context.expected_state = context.current_state;
    context.step_elapsed_ns = 0;
    context.fault_reset_asserted = false;
    return true;
  }

  context.transition_complete = false;
  if (context.expected_state != step.expected_state) {
    context.expected_state = step.expected_state;
    context.step_elapsed_ns = 0;
  } else if (!step.waiting_for_automatic_transition) {
    context.step_elapsed_ns = saturating_add(context.step_elapsed_ns, period_ns);
  }
  if (!step.waiting_for_automatic_transition &&
    context.step_elapsed_ns > step_timeout_ns_)
  {
    failure_code = ExecuteDriveCommand::Result::RESULT_TIMEOUT;
    std::ostringstream stream;
    stream << "Transition timed out for drive '" << context.joint_name << "' while waiting for '"
           << drive_state_name(step.expected_state) << "'";
    failure_message = stream.str();
    return false;
  }

  return true;
}

controller_interface::return_type Cia402Controller::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  const int64_t period_ns = std::max<int64_t>(0, period.nanoseconds());
  read_drive_feedback(time, false);

  const ActiveGoalPtr active_goal = *active_goal_buffer_.readFromRT();
  const uint8_t goal_state = active_goal ? active_goal->execution_state.load() :
    ActiveGoal::STATE_FINISHED;
  if (active_goal &&
    (goal_state == ActiveGoal::STATE_ACTIVE ||
    goal_state == ActiveGoal::STATE_CANCEL_REQUESTED))
  {
    if (!active_goal->started) {
      active_goal->started = true;
      active_goal->elapsed_ns.store(0);
      for (const std::size_t index : active_goal->selected_indices) {
        auto & context = drive_contexts_[index];
        context.transition_complete = false;
        context.expected_state = DriveState::UNKNOWN;
        context.fault_reset_asserted = false;
        context.waiting_for_mode = false;
        context.step_elapsed_ns = 0;
      }
    } else {
      active_goal->elapsed_ns.store(
        saturating_add(active_goal->elapsed_ns.load(), period_ns));
    }

    const int64_t elapsed_ns = active_goal->elapsed_ns.load();

    if (active_goal->execution_state.load() == ActiveGoal::STATE_CANCEL_REQUESTED) {
      stage_fallback_commands(active_goal->selected_indices);
      finish_goal(
        active_goal, false, ExecuteDriveCommand::Result::RESULT_CANCELLED,
        "Drive command canceled; fallback command '" +
        std::string(drive_command_name(fallback_command_)) + "' was staged", time, true);
    } else if (elapsed_ns > active_goal->timeout_ns) {
      stage_fallback_commands(active_goal->selected_indices);
      finish_goal(
        active_goal, false, ExecuteDriveCommand::Result::RESULT_TIMEOUT,
        "Drive command exceeded its overall timeout; fallback command '" +
        std::string(drive_command_name(fallback_command_)) + "' was staged", time);
    } else {
      bool all_complete = true;
      bool failed = false;
      bool transition_commanded = false;
      uint8_t failure_code = ExecuteDriveCommand::Result::RESULT_SUCCESS;
      std::string failure_message;

      for (const std::size_t index : active_goal->selected_indices) {
        const uint16_t previous_control_word = drive_contexts_[index].control_word;
        const DriveState previous_expected_state = drive_contexts_[index].expected_state;
        if (!advance_drive(
            drive_contexts_[index], index, active_goal->command, period_ns,
            failure_code, failure_message))
        {
          failed = true;
          break;
        }
        transition_commanded = transition_commanded ||
          drive_contexts_[index].control_word != previous_control_word ||
          drive_contexts_[index].expected_state != previous_expected_state;
        all_complete = all_complete && drive_contexts_[index].transition_complete;
      }

      if (failed) {
        stage_fallback_commands(active_goal->selected_indices);
        finish_goal(active_goal, false, failure_code, failure_message, time);
      } else if (all_complete) {
        finish_goal(
          active_goal, true, ExecuteDriveCommand::Result::RESULT_SUCCESS,
          "All selected drives completed command '" +
          std::string(drive_command_name(active_goal->command)) + "'", time);
      } else if (active_goal->command == DriveCommand::FAULT_RESET) {
        active_goal->phase.store(ExecuteDriveCommand::Feedback::PHASE_RESETTING_FAULT);
      } else {
        const bool waiting_for_mode = std::any_of(
          active_goal->selected_indices.begin(), active_goal->selected_indices.end(),
          [this](const std::size_t index) {return drive_contexts_[index].waiting_for_mode;});
        active_goal->phase.store(
          waiting_for_mode ? ExecuteDriveCommand::Feedback::PHASE_SETTING_MODE :
          (transition_commanded ?
          ExecuteDriveCommand::Feedback::PHASE_COMMANDING_TRANSITION :
          ExecuteDriveCommand::Feedback::PHASE_WAITING_FOR_FEEDBACK));
      }
    }
  }

  publish_drive_states(time, period_ns);
  update_non_realtime_snapshots(time);
  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse Cia402Controller::goal_callback(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const ExecuteDriveCommand::Goal> goal,
  const uint64_t activation_generation)
{
  if (!accepting_goals_.load() ||
    activation_generation_.load() != activation_generation)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!is_valid_drive_command(goal->command)) {
    RCLCPP_WARN(
      get_node()->get_logger(), "Rejecting goal with unknown drive command %u", goal->command);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!valid_goal_timeout(goal->timeout)) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting goal with an invalid negative duration");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::unordered_set<std::string> requested_joints;
  for (const auto & joint_name : goal->joint_names) {
    if (!requested_joints.insert(joint_name).second ||
      std::find(joint_names_.begin(), joint_names_.end(), joint_name) == joint_names_.end())
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Rejecting goal with duplicate or unknown joint '%s'",
        joint_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  uint64_t expected_generation = 0;
  if (!reserved_generation_.compare_exchange_strong(
      expected_generation, activation_generation))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting goal because another drive command is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!accepting_goals_.load() ||
    activation_generation_.load() != activation_generation)
  {
    release_goal_reservation(activation_generation);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void Cia402Controller::accepted_callback(
  std::shared_ptr<GoalHandle> goal_handle,
  const uint64_t activation_generation)
{
  std::lock_guard<std::mutex> callback_lock(action_callback_mutex_);
  if (!accepting_goals_.load() ||
    activation_generation_.load() != activation_generation ||
    reserved_generation_.load() != activation_generation)
  {
    auto result = std::make_shared<ExecuteDriveCommand::Result>();
    result->success = false;
    result->result_code = ExecuteDriveCommand::Result::RESULT_REJECTED;
    result->message = "Controller became inactive while accepting the drive command";
    goal_handle->abort(result);
    release_goal_reservation(activation_generation);
    return;
  }

  try {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ExecuteDriveCommand::Result>();
    auto realtime_goal = std::make_shared<RealtimeGoalHandle>(
      goal_handle, result, nullptr, get_node()->get_logger());
    auto active_goal = std::make_shared<ActiveGoal>();
    active_goal->handle = realtime_goal;
    active_goal->activation_generation = activation_generation;
    active_goal->command = to_drive_command(goal->command);
    active_goal->timeout_ns = timeout_to_nanoseconds(goal->timeout, default_timeout_ns_);

    if (goal->joint_names.empty()) {
      active_goal->selected_indices.resize(joint_names_.size());
      for (std::size_t index = 0; index < joint_names_.size(); ++index) {
        active_goal->selected_indices[index] = index;
      }
    } else {
      active_goal->selected_indices.reserve(goal->joint_names.size());
      for (const auto & requested_joint : goal->joint_names) {
        const auto iterator = std::find(joint_names_.begin(), joint_names_.end(), requested_joint);
        active_goal->selected_indices.push_back(
          static_cast<std::size_t>(std::distance(joint_names_.begin(), iterator)));
      }
    }

    result->final_states.resize(active_goal->selected_indices.size());
    realtime_goal->execute();
    active_goal_buffer_.writeFromNonRT(active_goal);

    RCLCPP_INFO(
      get_node()->get_logger(), "Accepted '%s' command for %zu drive(s)",
      drive_command_name(active_goal->command), active_goal->selected_indices.size());
  } catch (const std::exception & exception) {
    auto result = std::make_shared<ExecuteDriveCommand::Result>();
    result->success = false;
    result->result_code = ExecuteDriveCommand::Result::RESULT_REJECTED;
    result->message = std::string("Failed to start drive command: ") + exception.what();
    goal_handle->abort(result);
    release_goal_reservation(activation_generation);
  }
}

rclcpp_action::CancelResponse Cia402Controller::cancel_callback(
  const std::shared_ptr<GoalHandle> goal_handle,
  const uint64_t activation_generation)
{
  if (activation_generation_.load() != activation_generation) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  const ActiveGoalPtr active_goal = *active_goal_buffer_.readFromNonRT();
  if (!active_goal || active_goal->activation_generation != activation_generation ||
    active_goal->handle->gh_ != goal_handle)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }
  uint8_t expected_state = ActiveGoal::STATE_ACTIVE;
  if (!active_goal->execution_state.compare_exchange_strong(
      expected_state, ActiveGoal::STATE_CANCEL_REQUESTED))
  {
    return rclcpp_action::CancelResponse::REJECT;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Cia402Controller::monitor_goal(const ActiveGoalPtr & active_goal)
{
  std::lock_guard<std::mutex> callback_lock(action_callback_mutex_);
  if (!active_goal) {
    return;
  }
  const ActiveGoalPtr current_goal = *active_goal_buffer_.readFromNonRT();
  if (current_goal != active_goal) {
    return;
  }

  const uint8_t goal_state = active_goal->execution_state.load();
  if (goal_state == ActiveGoal::STATE_ACTIVE ||
    goal_state == ActiveGoal::STATE_CANCEL_REQUESTED)
  {
    auto feedback = std::make_shared<ExecuteDriveCommand::Feedback>();
    feedback->phase = active_goal->phase.load();
    feedback->elapsed = nanoseconds_to_duration_message(active_goal->elapsed_ns.load());

    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    feedback->states.reserve(active_goal->selected_indices.size());
    for (const std::size_t index : active_goal->selected_indices) {
      feedback->states.push_back(make_drive_state_message(non_realtime_snapshots_[index]));
    }
    active_goal->handle->setFeedback(feedback);
  }

  active_goal->handle->runNonRealtime();
  if (active_goal->execution_state.load() == ActiveGoal::STATE_FINISHED &&
    !active_goal->handle->gh_->is_active())
  {
    bool expected = false;
    if (active_goal->reservation_released.compare_exchange_strong(expected, true)) {
      const ActiveGoalPtr current_goal = *active_goal_buffer_.readFromNonRT();
      if (current_goal == active_goal) {
        active_goal_buffer_.writeFromNonRT(ActiveGoalPtr{});
        release_goal_reservation(active_goal->activation_generation);
      }
    }
  }
}

void Cia402Controller::release_goal_reservation(const uint64_t activation_generation)
{
  uint64_t expected_generation = activation_generation;
  reserved_generation_.compare_exchange_strong(expected_generation, 0);
}

void Cia402Controller::stage_fallback_commands(
  const std::vector<std::size_t> & selected_indices)
{
  for (const std::size_t index : selected_indices) {
    if (index >= drive_contexts_.size() || index >= control_word_interfaces_.size()) {
      continue;
    }
    auto & context = drive_contexts_[index];
    const TransitionStep fallback_step = calculate_transition_step(
      context.current_state, fallback_command_);
    context.control_word = fallback_step.valid ?
      fallback_step.control_word : CONTROLWORD_DISABLE_VOLTAGE;
    context.expected_state = fallback_step.valid ?
      fallback_step.expected_state : DriveState::SWITCH_ON_DISABLED;
    context.transition_complete = false;
    context.fault_reset_asserted = false;
    context.waiting_for_mode = false;
    context.step_elapsed_ns = 0;
    control_word_interfaces_[index].get().set_value(context.control_word);
  }
}

void Cia402Controller::finish_goal(
  const ActiveGoalPtr & active_goal,
  const bool success,
  const uint8_t result_code,
  const std::string & message,
  const rclcpp::Time & time,
  const bool canceled)
{
  if (!active_goal) {
    return;
  }

  uint8_t observed_state = active_goal->execution_state.load();
  while (observed_state != ActiveGoal::STATE_TERMINATING &&
    observed_state != ActiveGoal::STATE_FINISHED &&
    !active_goal->execution_state.compare_exchange_weak(
      observed_state, ActiveGoal::STATE_TERMINATING))
  {
  }
  if (observed_state == ActiveGoal::STATE_TERMINATING ||
    observed_state == ActiveGoal::STATE_FINISHED)
  {
    return;
  }

  const bool cancellation_won_race =
    observed_state == ActiveGoal::STATE_CANCEL_REQUESTED;
  const bool effective_canceled = canceled || cancellation_won_race;
  const bool effective_success = success && !effective_canceled;
  const uint8_t effective_result_code = effective_canceled ?
    ExecuteDriveCommand::Result::RESULT_CANCELLED : result_code;
  const std::string effective_message = effective_canceled ?
    "Drive command canceled; fallback command '" +
    std::string(drive_command_name(fallback_command_)) + "' was staged" : message;
  if (effective_canceled) {
    stage_fallback_commands(active_goal->selected_indices);
  }

  auto result = active_goal->handle->preallocated_result_;
  result->success = effective_success;
  result->result_code = effective_result_code;
  result->message = effective_message;
  result->final_states.clear();
  result->final_states.reserve(active_goal->selected_indices.size());
  for (const std::size_t index : active_goal->selected_indices) {
    result->final_states.push_back(make_drive_state_message(make_drive_snapshot(index, time)));
  }

  if (effective_success) {
    active_goal->handle->setSucceeded(result);
    RCLCPP_INFO(get_node()->get_logger(), "%s", effective_message.c_str());
  } else if (effective_canceled) {
    active_goal->handle->setCanceled(result);
    RCLCPP_WARN(get_node()->get_logger(), "%s", effective_message.c_str());
  } else {
    active_goal->handle->setAborted(result);
    RCLCPP_ERROR(get_node()->get_logger(), "%s", effective_message.c_str());
  }
  active_goal->execution_state.store(ActiveGoal::STATE_FINISHED);
}

void Cia402Controller::abort_active_goal_on_deactivate()
{
  const ActiveGoalPtr active_goal = *active_goal_buffer_.readFromNonRT();
  if (!active_goal) {
    return;
  }

  const uint8_t execution_state = active_goal->execution_state.load();
  if (execution_state != ActiveGoal::STATE_FINISHED &&
    execution_state != ActiveGoal::STATE_TERMINATING)
  {
    stage_fallback_commands(active_goal->selected_indices);
    finish_goal(
      active_goal, false, ExecuteDriveCommand::Result::RESULT_REJECTED,
      "Controller deactivated before the drive command completed; fallback command '" +
      std::string(drive_command_name(fallback_command_)) + "' was staged",
      get_node()->now());
  }

  // RealtimeServerGoalHandle sends queued results from this non-real-time
  // method. Flush even an already-finished goal before destroying its timer.
  active_goal->handle->runNonRealtime();
}

DriveSnapshot Cia402Controller::make_drive_snapshot(
  const std::size_t drive_index,
  const rclcpp::Time & time) const
{
  const auto & context = drive_contexts_[drive_index];
  DriveSnapshot snapshot;
  snapshot.stamp = static_cast<builtin_interfaces::msg::Time>(time);
  snapshot.joint_name = context.joint_name;
  snapshot.status_word = context.status_word;
  snapshot.control_word = context.control_word;
  snapshot.modes_of_operation_display = context.modes_of_operation_display;
  snapshot.modes_of_operation = context.modes_of_operation;
  snapshot.state = context.current_state;
  snapshot.feedback_valid = context.feedback_valid;
  return snapshot;
}

cia402_interfaces::msg::DriveState Cia402Controller::make_drive_state_message(
  const DriveSnapshot & snapshot)
{
  cia402_interfaces::msg::DriveState message;
  message.stamp = snapshot.stamp;
  message.joint_name = snapshot.joint_name;
  message.status_word = snapshot.status_word;
  message.mode_of_operation_display = snapshot.modes_of_operation_display;
  message.feedback_valid = snapshot.feedback_valid;
  message.state = static_cast<uint8_t>(snapshot.state);
  message.commanded_control_word = snapshot.control_word;
  message.commanded_mode_of_operation = snapshot.modes_of_operation;

  StatusWordFlags flags{};
  if (snapshot.feedback_valid) {
    flags = decode_status_word_flags(snapshot.status_word);
  }
  message.ready_to_switch_on = flags.ready_to_switch_on;
  message.switched_on = flags.switched_on;
  message.operation_enabled = flags.operation_enabled;
  message.fault = flags.fault;
  message.voltage_enabled = flags.voltage_enabled;
  message.quick_stop_active = flags.quick_stop_active;
  message.switch_on_disabled = flags.switch_on_disabled;
  message.warning = flags.warning;
  message.remote = flags.remote;
  message.target_reached = flags.target_reached;
  message.internal_limit_active = flags.internal_limit_active;
  return message;
}

void Cia402Controller::publish_drive_states(
  const rclcpp::Time & time,
  const int64_t period_ns)
{
  if (!realtime_state_publisher_) {
    return;
  }
  state_publish_elapsed_ns_ = saturating_add(state_publish_elapsed_ns_, period_ns);
  if (state_published_ && state_publish_elapsed_ns_ < feedback_period_ns_) {
    return;
  }
  if (!realtime_state_publisher_->trylock()) {
    return;
  }

  auto & message = realtime_state_publisher_->msg_;
  message.stamp = static_cast<builtin_interfaces::msg::Time>(time);
  if (message.states.size() != drive_contexts_.size()) {
    message.states.resize(drive_contexts_.size());
  }
  for (std::size_t index = 0; index < drive_contexts_.size(); ++index) {
    message.states[index] = make_drive_state_message(make_drive_snapshot(index, time));
  }
  realtime_state_publisher_->unlockAndPublish();
  state_published_ = true;
  state_publish_elapsed_ns_ = 0;
}

void Cia402Controller::update_non_realtime_snapshots(const rclcpp::Time & time)
{
  if (!snapshot_mutex_.try_lock()) {
    return;
  }
  for (std::size_t index = 0; index < drive_contexts_.size(); ++index) {
    non_realtime_snapshots_[index] = make_drive_snapshot(index, time);
  }
  snapshot_mutex_.unlock();
}

}  // namespace cia402_controller

PLUGINLIB_EXPORT_CLASS(
  cia402_controller::Cia402Controller,
  controller_interface::ControllerInterface)
