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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>

#include "cia402_controller/cia402_controller.hpp"

namespace cia402_controller
{
namespace
{

class TestableCia402Controller : public Cia402Controller
{
public:
  using Cia402Controller::advance_drive;
  using Cia402Controller::drive_contexts_;
  using Cia402Controller::read_drive_feedback;
};

class Cia402ControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argument_count = 0;
      rclcpp::init(argument_count, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void configure_and_activate()
  {
    ASSERT_EQ(
      controller_.init("test_cia402_controller"),
      controller_interface::return_type::OK);
    controller_.get_node()->set_parameter(rclcpp::Parameter("joints", joint_names_));
    controller_.get_node()->set_parameter(
      rclcpp::Parameter("default_mode_of_operation", 8));
    controller_.get_node()->set_parameter(rclcpp::Parameter("step_timeout", 1.0));
    controller_.get_node()->set_parameter(rclcpp::Parameter("feedback_rate", 20.0));
    ASSERT_EQ(
      controller_.on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);

    command_handles_.emplace_back("joint1", "control_word", &control_word_);
    command_handles_.emplace_back("joint1", "modes_of_operation", &mode_command_);
    state_handles_.emplace_back("joint1", "status_word", &status_word_);
    state_handles_.emplace_back(
      "joint1", "modes_of_operation_display", &mode_display_);

    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    for (auto & command_handle : command_handles_) {
      command_interfaces.emplace_back(command_handle);
    }
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    for (auto & state_handle : state_handles_) {
      state_interfaces.emplace_back(state_handle);
    }
    controller_.assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
    ASSERT_EQ(
      controller_.on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  void refresh_feedback(const int64_t nanoseconds)
  {
    ASSERT_TRUE(
      controller_.read_drive_feedback(
        rclcpp::Time(nanoseconds, RCL_ROS_TIME), true));
  }

  bool advance(const DriveCommand command, const int64_t period_nanoseconds)
  {
    uint8_t failure_code = 0;
    std::string failure_message;
    return controller_.advance_drive(
      controller_.drive_contexts_.at(0), 0, command,
      period_nanoseconds, failure_code, failure_message);
  }

  TestableCia402Controller controller_;
  std::vector<std::string> joint_names_{"joint1"};
  double control_word_{0.0};
  double mode_command_{0.0};
  double status_word_{0x0040};
  double mode_display_{8.0};
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
};

TEST_F(Cia402ControllerTest, ClaimsTheDriverInterfaceContract)
{
  ASSERT_EQ(
    controller_.init("test_cia402_interface_configuration"),
    controller_interface::return_type::OK);
  controller_.get_node()->set_parameter(rclcpp::Parameter("joints", joint_names_));
  ASSERT_EQ(
    controller_.on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  const auto command_configuration = controller_.command_interface_configuration();
  EXPECT_EQ(
    command_configuration.type,
    controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(
    command_configuration.names,
    (std::vector<std::string>{"joint1/control_word", "joint1/modes_of_operation"}));

  const auto state_configuration = controller_.state_interface_configuration();
  EXPECT_EQ(
    state_configuration.names,
    (std::vector<std::string>{"joint1/status_word", "joint1/modes_of_operation_display"}));
}

TEST_F(Cia402ControllerTest, LoadsThroughTheControllerInterfacePlugin)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  const auto controller = loader.createSharedInstance(
    "cia402_controller/Cia402Controller");
  EXPECT_NE(controller, nullptr);
}

TEST_F(Cia402ControllerTest, AdvancesEnableSequenceAndWaitsForModeConfirmation)
{
  configure_and_activate();
  refresh_feedback(100000000);

  ASSERT_TRUE(advance(DriveCommand::ENABLE_OPERATION, 100000000));
  EXPECT_EQ(control_word_, CONTROLWORD_SHUTDOWN);
  EXPECT_EQ(mode_command_, 8.0);

  status_word_ = 0x0021;
  refresh_feedback(200000000);
  ASSERT_TRUE(advance(DriveCommand::ENABLE_OPERATION, 200000000));
  EXPECT_EQ(control_word_, CONTROLWORD_SWITCH_ON);

  status_word_ = 0x0023;
  mode_display_ = 0.0;
  refresh_feedback(300000000);
  ASSERT_TRUE(advance(DriveCommand::ENABLE_OPERATION, 300000000));
  EXPECT_EQ(control_word_, CONTROLWORD_SWITCH_ON);
  EXPECT_TRUE(controller_.drive_contexts_.at(0).waiting_for_mode);

  mode_display_ = 8.0;
  refresh_feedback(400000000);
  ASSERT_TRUE(advance(DriveCommand::ENABLE_OPERATION, 400000000));
  EXPECT_EQ(control_word_, CONTROLWORD_ENABLE_OPERATION);

  status_word_ = 0x0027;
  refresh_feedback(500000000);
  ASSERT_TRUE(advance(DriveCommand::ENABLE_OPERATION, 500000000));
  EXPECT_TRUE(controller_.drive_contexts_.at(0).transition_complete);

  EXPECT_EQ(
    controller_.on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(control_word_, CONTROLWORD_QUICK_STOP);
}

TEST_F(Cia402ControllerTest, ActivationPreservesAnEnabledDriveBeforeAnyAction)
{
  status_word_ = 0x0027;
  mode_display_ = 8.0;
  configure_and_activate();

  EXPECT_EQ(control_word_, CONTROLWORD_ENABLE_OPERATION);
  EXPECT_EQ(mode_command_, 8.0);

  EXPECT_EQ(
    controller_.on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(control_word_, CONTROLWORD_QUICK_STOP);
}

TEST_F(Cia402ControllerTest, PulsesAndClearsFaultReset)
{
  status_word_ = 0x0008;
  configure_and_activate();
  refresh_feedback(100000000);

  ASSERT_TRUE(advance(DriveCommand::FAULT_RESET, 100000000));
  EXPECT_EQ(control_word_, CONTROLWORD_FAULT_RESET);

  ASSERT_TRUE(advance(DriveCommand::FAULT_RESET, 200000000));
  EXPECT_EQ(control_word_, CONTROLWORD_DISABLE_VOLTAGE);

  status_word_ = 0x0040;
  refresh_feedback(300000000);
  ASSERT_TRUE(advance(DriveCommand::FAULT_RESET, 300000000));
  EXPECT_TRUE(controller_.drive_contexts_.at(0).transition_complete);
  EXPECT_EQ(control_word_, CONTROLWORD_DISABLE_VOLTAGE);
}

TEST_F(Cia402ControllerTest, StepTimeoutAccumulatesControllerPeriods)
{
  configure_and_activate();
  refresh_feedback(100000000);

  ASSERT_TRUE(advance(DriveCommand::ENABLE_OPERATION, 0));
  ASSERT_TRUE(advance(DriveCommand::ENABLE_OPERATION, 600000000));
  EXPECT_FALSE(advance(DriveCommand::ENABLE_OPERATION, 600000000));
}

TEST_F(Cia402ControllerTest, ExecutesSemanticCommandThroughActionServer)
{
  using namespace std::chrono_literals;
  using Action = cia402_interfaces::action::ExecuteDriveCommand;

  configure_and_activate();

  auto client_node = std::make_shared<rclcpp::Node>("cia402_action_test_client");
  auto action_client = rclcpp_action::create_client<Action>(
    client_node, "/test_cia402_controller/execute_drive_command");
  std::atomic<std::size_t> state_message_count{0};
  uint8_t last_published_state = cia402_interfaces::msg::DriveState::STATE_UNKNOWN;
  auto state_subscription =
    client_node->create_subscription<cia402_interfaces::msg::DriveStateArray>(
    "/test_cia402_controller/drive_states", rclcpp::QoS(1).reliable(),
    [&state_message_count, &last_published_state](
      const cia402_interfaces::msg::DriveStateArray::SharedPtr message)
    {
      state_message_count.fetch_add(1);
      if (!message->states.empty()) {
        last_published_state = message->states.front().state;
      }
    });
  ASSERT_NE(state_subscription, nullptr);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_.get_node()->get_node_base_interface());
  executor.add_node(client_node);
  ASSERT_TRUE(action_client->wait_for_action_server(1s));

  std::atomic<std::size_t> feedback_count{0};
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback =
    [&feedback_count](
    rclcpp_action::ClientGoalHandle<Action>::SharedPtr,
    const std::shared_ptr<const Action::Feedback>)
    {
      feedback_count.fetch_add(1);
    };

  Action::Goal goal;
  goal.command = Action::Goal::COMMAND_ENABLE_OPERATION;
  goal.timeout.sec = 5;
  auto goal_future = action_client->async_send_goal(goal, options);
  ASSERT_EQ(
    executor.spin_until_future_complete(goal_future, 1s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  const auto result_future = action_client->async_get_result(goal_handle);

  EXPECT_EQ(
    controller_.update(
      rclcpp::Time(1000000000LL, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(control_word_, CONTROLWORD_SHUTDOWN);

  std::this_thread::sleep_for(60ms);
  executor.spin_some();
  executor.spin_some();

  status_word_ = 0x0021;
  controller_.update(
    rclcpp::Time(1010000000LL, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
  EXPECT_EQ(control_word_, CONTROLWORD_SWITCH_ON);

  status_word_ = 0x0023;
  controller_.update(
    rclcpp::Time(1020000000LL, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
  EXPECT_EQ(control_word_, CONTROLWORD_ENABLE_OPERATION);

  status_word_ = 0x0027;
  controller_.update(
    rclcpp::Time(1030000000LL, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.05));

  ASSERT_EQ(
    executor.spin_until_future_complete(result_future, 1s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->result_code, Action::Result::RESULT_SUCCESS);
  ASSERT_EQ(wrapped_result.result->final_states.size(), 1U);
  EXPECT_EQ(
    wrapped_result.result->final_states.front().state,
    cia402_interfaces::msg::DriveState::STATE_OPERATION_ENABLED);
  EXPECT_GT(feedback_count.load(), 0U);
  executor.spin_some();
  EXPECT_GT(state_message_count.load(), 0U);
  EXPECT_EQ(
    last_published_state,
    cia402_interfaces::msg::DriveState::STATE_OPERATION_ENABLED);

  executor.remove_node(client_node);
  executor.remove_node(controller_.get_node()->get_node_base_interface());
  EXPECT_EQ(
    controller_.on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(Cia402ControllerTest, RejectsInvalidFeedbackAtActivation)
{
  status_word_ = std::numeric_limits<double>::quiet_NaN();

  ASSERT_EQ(
    controller_.init("test_cia402_invalid_feedback"),
    controller_interface::return_type::OK);
  controller_.get_node()->set_parameter(rclcpp::Parameter("joints", joint_names_));
  ASSERT_EQ(
    controller_.on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  command_handles_.emplace_back("joint1", "control_word", &control_word_);
  command_handles_.emplace_back("joint1", "modes_of_operation", &mode_command_);
  state_handles_.emplace_back("joint1", "status_word", &status_word_);
  state_handles_.emplace_back(
    "joint1", "modes_of_operation_display", &mode_display_);
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(command_handles_[0]);
  command_interfaces.emplace_back(command_handles_[1]);
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(state_handles_[0]);
  state_interfaces.emplace_back(state_handles_[1]);
  controller_.assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

  EXPECT_EQ(
    controller_.on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(Cia402ControllerTest, CancellationStagesTheConfiguredFallback)
{
  using namespace std::chrono_literals;
  using Action = cia402_interfaces::action::ExecuteDriveCommand;

  status_word_ = 0x0027;
  configure_and_activate();

  auto client_node = std::make_shared<rclcpp::Node>("cia402_cancel_test_client");
  auto action_client = rclcpp_action::create_client<Action>(
    client_node, "/test_cia402_controller/execute_drive_command");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_.get_node()->get_node_base_interface());
  executor.add_node(client_node);
  ASSERT_TRUE(action_client->wait_for_action_server(1s));

  Action::Goal goal;
  goal.command = Action::Goal::COMMAND_SHUTDOWN;
  goal.timeout.sec = 5;
  auto goal_future = action_client->async_send_goal(goal);
  ASSERT_EQ(
    executor.spin_until_future_complete(goal_future, 1s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  const auto result_future = action_client->async_get_result(goal_handle);

  auto competing_goal_future = action_client->async_send_goal(goal);
  ASSERT_EQ(
    executor.spin_until_future_complete(competing_goal_future, 1s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(competing_goal_future.get(), nullptr);

  auto cancel_future = action_client->async_cancel_goal(goal_handle);
  ASSERT_EQ(
    executor.spin_until_future_complete(cancel_future, 1s),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(cancel_future.get()->goals_canceling.empty());

  controller_.update(
    rclcpp::Time(1000000000LL, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
  EXPECT_EQ(control_word_, CONTROLWORD_QUICK_STOP);

  ASSERT_EQ(
    executor.spin_until_future_complete(result_future, 1s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
  EXPECT_EQ(wrapped_result.result->result_code, Action::Result::RESULT_CANCELLED);

  executor.remove_node(client_node);
  executor.remove_node(controller_.get_node()->get_node_base_interface());
  EXPECT_EQ(
    controller_.on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

}  // namespace
}  // namespace cia402_controller
