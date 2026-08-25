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

#ifndef CIA402_CONTROLLER__CIA402_STATE_MACHINE_HPP_
#define CIA402_CONTROLLER__CIA402_STATE_MACHINE_HPP_

#include <cstdint>

namespace cia402_controller
{

// Values intentionally match cia402_interfaces/msg/DriveState.
enum class DriveState : uint8_t
{
  NOT_READY_TO_SWITCH_ON = 0,
  SWITCH_ON_DISABLED = 1,
  READY_TO_SWITCH_ON = 2,
  SWITCHED_ON = 3,
  OPERATION_ENABLED = 4,
  QUICK_STOP_ACTIVE = 5,
  FAULT_REACTION_ACTIVE = 6,
  FAULT = 7,
  UNKNOWN = 255
};

// Values intentionally match cia402_interfaces/action/ExecuteDriveCommand.
enum class DriveCommand : uint8_t
{
  SHUTDOWN = 1,
  SWITCH_ON = 2,
  ENABLE_OPERATION = 3,
  DISABLE_OPERATION = 4,
  DISABLE_VOLTAGE = 5,
  QUICK_STOP = 6,
  FAULT_RESET = 7
};

constexpr uint16_t CONTROLWORD_SHUTDOWN = 0x0006;
constexpr uint16_t CONTROLWORD_SWITCH_ON = 0x0007;
constexpr uint16_t CONTROLWORD_ENABLE_OPERATION = 0x000F;
constexpr uint16_t CONTROLWORD_DISABLE_VOLTAGE = 0x0000;
constexpr uint16_t CONTROLWORD_QUICK_STOP = 0x0002;
constexpr uint16_t CONTROLWORD_FAULT_RESET = 0x0080;

struct StatusWordFlags
{
  bool ready_to_switch_on;
  bool switched_on;
  bool operation_enabled;
  bool fault;
  bool voltage_enabled;
  bool quick_stop_active;
  bool switch_on_disabled;
  bool warning;
  bool remote;
  bool target_reached;
  bool internal_limit_active;
};

struct TransitionStep
{
  uint16_t control_word{CONTROLWORD_DISABLE_VOLTAGE};
  DriveState expected_state{DriveState::UNKNOWN};
  bool goal_reached{false};
  bool valid{false};
  bool waiting_for_automatic_transition{false};
};

DriveState decode_status_word(uint16_t status_word);

StatusWordFlags decode_status_word_flags(uint16_t status_word);

bool is_valid_drive_command(uint8_t command);

DriveCommand to_drive_command(uint8_t command);

const char * drive_state_name(DriveState state);

const char * drive_command_name(DriveCommand command);

TransitionStep calculate_transition_step(
  DriveState current_state,
  DriveCommand requested_command);

}  // namespace cia402_controller

#endif  // CIA402_CONTROLLER__CIA402_STATE_MACHINE_HPP_
