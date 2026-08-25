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

#include <array>
#include <cstdint>

#include <cia402_interfaces/action/execute_drive_command.hpp>
#include <cia402_interfaces/msg/drive_state.hpp>

#include "cia402_controller/cia402_state_machine.hpp"

namespace cia402_controller
{
namespace
{

TEST(Cia402StateMachine, InternalValuesMatchRosInterfaces)
{
  using StateMessage = cia402_interfaces::msg::DriveState;
  using Action = cia402_interfaces::action::ExecuteDriveCommand;

  EXPECT_EQ(
    static_cast<uint8_t>(DriveState::NOT_READY_TO_SWITCH_ON),
    StateMessage::STATE_NOT_READY_TO_SWITCH_ON);
  EXPECT_EQ(
    static_cast<uint8_t>(DriveState::SWITCH_ON_DISABLED),
    StateMessage::STATE_SWITCH_ON_DISABLED);
  EXPECT_EQ(
    static_cast<uint8_t>(DriveState::READY_TO_SWITCH_ON),
    StateMessage::STATE_READY_TO_SWITCH_ON);
  EXPECT_EQ(static_cast<uint8_t>(DriveState::SWITCHED_ON), StateMessage::STATE_SWITCHED_ON);
  EXPECT_EQ(
    static_cast<uint8_t>(DriveState::OPERATION_ENABLED),
    StateMessage::STATE_OPERATION_ENABLED);
  EXPECT_EQ(
    static_cast<uint8_t>(DriveState::QUICK_STOP_ACTIVE),
    StateMessage::STATE_QUICK_STOP_ACTIVE);
  EXPECT_EQ(
    static_cast<uint8_t>(DriveState::FAULT_REACTION_ACTIVE),
    StateMessage::STATE_FAULT_REACTION_ACTIVE);
  EXPECT_EQ(static_cast<uint8_t>(DriveState::FAULT), StateMessage::STATE_FAULT);
  EXPECT_EQ(static_cast<uint8_t>(DriveState::UNKNOWN), StateMessage::STATE_UNKNOWN);

  EXPECT_EQ(static_cast<uint8_t>(DriveCommand::SHUTDOWN), Action::Goal::COMMAND_SHUTDOWN);
  EXPECT_EQ(static_cast<uint8_t>(DriveCommand::SWITCH_ON), Action::Goal::COMMAND_SWITCH_ON);
  EXPECT_EQ(
    static_cast<uint8_t>(DriveCommand::ENABLE_OPERATION),
    Action::Goal::COMMAND_ENABLE_OPERATION);
  EXPECT_EQ(
    static_cast<uint8_t>(DriveCommand::DISABLE_OPERATION),
    Action::Goal::COMMAND_DISABLE_OPERATION);
  EXPECT_EQ(
    static_cast<uint8_t>(DriveCommand::DISABLE_VOLTAGE),
    Action::Goal::COMMAND_DISABLE_VOLTAGE);
  EXPECT_EQ(static_cast<uint8_t>(DriveCommand::QUICK_STOP), Action::Goal::COMMAND_QUICK_STOP);
  EXPECT_EQ(static_cast<uint8_t>(DriveCommand::FAULT_RESET), Action::Goal::COMMAND_FAULT_RESET);
}

TEST(Cia402StateMachine, DecodesCanonicalStatusWords)
{
  const std::array<std::pair<uint16_t, DriveState>, 8> cases{{
    {0x0000, DriveState::NOT_READY_TO_SWITCH_ON},
    {0x0040, DriveState::SWITCH_ON_DISABLED},
    {0x0021, DriveState::READY_TO_SWITCH_ON},
    {0x0023, DriveState::SWITCHED_ON},
    {0x0027, DriveState::OPERATION_ENABLED},
    {0x0007, DriveState::QUICK_STOP_ACTIVE},
    {0x000F, DriveState::FAULT_REACTION_ACTIVE},
    {0x0008, DriveState::FAULT}}};

  for (const auto & test_case : cases) {
    EXPECT_EQ(decode_status_word(test_case.first), test_case.second);
    EXPECT_EQ(
      decode_status_word(static_cast<uint16_t>(test_case.first | 0xA590)),
      test_case.second);
  }
  EXPECT_EQ(decode_status_word(0x006F), DriveState::UNKNOWN);
}

TEST(Cia402StateMachine, DecodesQuickStopActiveFromTheCompleteStatePattern)
{
  EXPECT_TRUE(decode_status_word_flags(0x0007).quick_stop_active);
  EXPECT_FALSE(decode_status_word_flags(0x0000).quick_stop_active);
  EXPECT_FALSE(decode_status_word_flags(0x0008).quick_stop_active);
  EXPECT_FALSE(decode_status_word_flags(1U << 5).quick_stop_active);
}

TEST(Cia402StateMachine, BuildsEnableOperationSequence)
{
  auto step = calculate_transition_step(
    DriveState::SWITCH_ON_DISABLED, DriveCommand::ENABLE_OPERATION);
  ASSERT_TRUE(step.valid);
  EXPECT_EQ(step.control_word, CONTROLWORD_SHUTDOWN);
  EXPECT_EQ(step.expected_state, DriveState::READY_TO_SWITCH_ON);
  EXPECT_FALSE(step.goal_reached);

  step = calculate_transition_step(
    DriveState::READY_TO_SWITCH_ON, DriveCommand::ENABLE_OPERATION);
  ASSERT_TRUE(step.valid);
  EXPECT_EQ(step.control_word, CONTROLWORD_SWITCH_ON);
  EXPECT_EQ(step.expected_state, DriveState::SWITCHED_ON);

  step = calculate_transition_step(DriveState::SWITCHED_ON, DriveCommand::ENABLE_OPERATION);
  ASSERT_TRUE(step.valid);
  EXPECT_EQ(step.control_word, CONTROLWORD_ENABLE_OPERATION);
  EXPECT_EQ(step.expected_state, DriveState::OPERATION_ENABLED);

  step = calculate_transition_step(
    DriveState::OPERATION_ENABLED, DriveCommand::ENABLE_OPERATION);
  EXPECT_TRUE(step.valid);
  EXPECT_TRUE(step.goal_reached);
}

TEST(Cia402StateMachine, SupportsIdempotentSafeCommands)
{
  auto step = calculate_transition_step(
    DriveState::READY_TO_SWITCH_ON, DriveCommand::DISABLE_OPERATION);
  EXPECT_TRUE(step.valid);
  EXPECT_TRUE(step.goal_reached);
  EXPECT_EQ(step.control_word, CONTROLWORD_SHUTDOWN);

  step = calculate_transition_step(
    DriveState::SWITCH_ON_DISABLED, DriveCommand::DISABLE_VOLTAGE);
  EXPECT_TRUE(step.valid);
  EXPECT_TRUE(step.goal_reached);
  EXPECT_EQ(step.control_word, CONTROLWORD_DISABLE_VOLTAGE);
}

TEST(Cia402StateMachine, AcceptsBothQuickStopTerminalStates)
{
  auto step = calculate_transition_step(
    DriveState::OPERATION_ENABLED, DriveCommand::QUICK_STOP);
  EXPECT_TRUE(step.valid);
  EXPECT_FALSE(step.goal_reached);
  EXPECT_EQ(step.control_word, CONTROLWORD_QUICK_STOP);

  step = calculate_transition_step(DriveState::QUICK_STOP_ACTIVE, DriveCommand::QUICK_STOP);
  EXPECT_TRUE(step.valid);
  EXPECT_TRUE(step.goal_reached);

  step = calculate_transition_step(DriveState::SWITCH_ON_DISABLED, DriveCommand::QUICK_STOP);
  EXPECT_TRUE(step.valid);
  EXPECT_TRUE(step.goal_reached);
}

TEST(Cia402StateMachine, FaultResetOnlyAppliesToFaultPath)
{
  auto step = calculate_transition_step(DriveState::FAULT, DriveCommand::FAULT_RESET);
  EXPECT_TRUE(step.valid);
  EXPECT_FALSE(step.goal_reached);
  EXPECT_EQ(step.control_word, CONTROLWORD_FAULT_RESET);
  EXPECT_EQ(step.expected_state, DriveState::SWITCH_ON_DISABLED);

  step = calculate_transition_step(
    DriveState::FAULT_REACTION_ACTIVE, DriveCommand::FAULT_RESET);
  EXPECT_TRUE(step.valid);
  EXPECT_TRUE(step.waiting_for_automatic_transition);
  EXPECT_EQ(step.expected_state, DriveState::FAULT);

  EXPECT_FALSE(
    calculate_transition_step(
      DriveState::OPERATION_ENABLED, DriveCommand::FAULT_RESET).valid);
  EXPECT_FALSE(
    calculate_transition_step(
      DriveState::FAULT, DriveCommand::ENABLE_OPERATION).valid);
}

TEST(Cia402StateMachine, ValidatesCommandValues)
{
  EXPECT_FALSE(is_valid_drive_command(0));
  EXPECT_TRUE(is_valid_drive_command(1));
  EXPECT_TRUE(is_valid_drive_command(7));
  EXPECT_FALSE(is_valid_drive_command(8));
  EXPECT_THROW(to_drive_command(0), std::invalid_argument);
}

}  // namespace
}  // namespace cia402_controller
