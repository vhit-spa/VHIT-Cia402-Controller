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

#include "cia402_controller/cia402_state_machine.hpp"

#include <stdexcept>

namespace cia402_controller
{

DriveState decode_status_word(const uint16_t status_word)
{
  switch (status_word & 0x006F) {
    case 0x0021:
      return DriveState::READY_TO_SWITCH_ON;
    case 0x0023:
      return DriveState::SWITCHED_ON;
    case 0x0027:
      return DriveState::OPERATION_ENABLED;
    case 0x0007:
      return DriveState::QUICK_STOP_ACTIVE;
    default:
      break;
  }

  switch (status_word & 0x004F) {
    case 0x0000:
      return DriveState::NOT_READY_TO_SWITCH_ON;
    case 0x0040:
      return DriveState::SWITCH_ON_DISABLED;
    case 0x000F:
      return DriveState::FAULT_REACTION_ACTIVE;
    case 0x0008:
      return DriveState::FAULT;
    default:
      return DriveState::UNKNOWN;
  }
}

StatusWordFlags decode_status_word_flags(const uint16_t status_word)
{
  const bool quick_stop_active =
    decode_status_word(status_word) == DriveState::QUICK_STOP_ACTIVE;
  return StatusWordFlags{
    (status_word & (1U << 0)) != 0,
    (status_word & (1U << 1)) != 0,
    (status_word & (1U << 2)) != 0,
    (status_word & (1U << 3)) != 0,
    (status_word & (1U << 4)) != 0,
    quick_stop_active,
    (status_word & (1U << 6)) != 0,
    (status_word & (1U << 7)) != 0,
    (status_word & (1U << 9)) != 0,
    (status_word & (1U << 10)) != 0,
    (status_word & (1U << 11)) != 0};
}

bool is_valid_drive_command(const uint8_t command)
{
  return command >= static_cast<uint8_t>(DriveCommand::SHUTDOWN) &&
         command <= static_cast<uint8_t>(DriveCommand::FAULT_RESET);
}

DriveCommand to_drive_command(const uint8_t command)
{
  if (!is_valid_drive_command(command)) {
    throw std::invalid_argument("Unknown CiA 402 drive command");
  }
  return static_cast<DriveCommand>(command);
}

const char * drive_state_name(const DriveState state)
{
  switch (state) {
    case DriveState::NOT_READY_TO_SWITCH_ON:
      return "not ready to switch on";
    case DriveState::SWITCH_ON_DISABLED:
      return "switch on disabled";
    case DriveState::READY_TO_SWITCH_ON:
      return "ready to switch on";
    case DriveState::SWITCHED_ON:
      return "switched on";
    case DriveState::OPERATION_ENABLED:
      return "operation enabled";
    case DriveState::QUICK_STOP_ACTIVE:
      return "quick stop active";
    case DriveState::FAULT_REACTION_ACTIVE:
      return "fault reaction active";
    case DriveState::FAULT:
      return "fault";
    case DriveState::UNKNOWN:
      return "unknown";
  }
  return "unknown";
}

const char * drive_command_name(const DriveCommand command)
{
  switch (command) {
    case DriveCommand::SHUTDOWN:
      return "shutdown";
    case DriveCommand::SWITCH_ON:
      return "switch on";
    case DriveCommand::ENABLE_OPERATION:
      return "enable operation";
    case DriveCommand::DISABLE_OPERATION:
      return "disable operation";
    case DriveCommand::DISABLE_VOLTAGE:
      return "disable voltage";
    case DriveCommand::QUICK_STOP:
      return "quick stop";
    case DriveCommand::FAULT_RESET:
      return "fault reset";
  }
  return "unknown";
}

TransitionStep calculate_transition_step(
  const DriveState current_state,
  const DriveCommand requested_command)
{
  if (current_state == DriveState::UNKNOWN) {
    return {};
  }

  // These states advance without a Controlword transition. Keep a safe output
  // and let the goal-level timeout bound the wait.
  if (current_state == DriveState::NOT_READY_TO_SWITCH_ON) {
    return TransitionStep{
      CONTROLWORD_DISABLE_VOLTAGE, DriveState::SWITCH_ON_DISABLED, false, true, true};
  }
  if (current_state == DriveState::FAULT_REACTION_ACTIVE) {
    return TransitionStep{
      CONTROLWORD_DISABLE_VOLTAGE, DriveState::FAULT, false,
      requested_command == DriveCommand::FAULT_RESET, true};
  }

  switch (requested_command) {
    case DriveCommand::SHUTDOWN:
      switch (current_state) {
        case DriveState::READY_TO_SWITCH_ON:
          return {CONTROLWORD_SHUTDOWN, DriveState::READY_TO_SWITCH_ON, true, true, false};
        case DriveState::SWITCH_ON_DISABLED:
          return {CONTROLWORD_SHUTDOWN, DriveState::READY_TO_SWITCH_ON, false, true, false};
        case DriveState::SWITCHED_ON:
        case DriveState::OPERATION_ENABLED:
          return {CONTROLWORD_SHUTDOWN, DriveState::READY_TO_SWITCH_ON, false, true, false};
        case DriveState::QUICK_STOP_ACTIVE:
          return {
            CONTROLWORD_DISABLE_VOLTAGE, DriveState::SWITCH_ON_DISABLED, false, true, false};
        default:
          return {};
      }

    case DriveCommand::SWITCH_ON:
      switch (current_state) {
        case DriveState::SWITCHED_ON:
          return {CONTROLWORD_SWITCH_ON, DriveState::SWITCHED_ON, true, true, false};
        case DriveState::READY_TO_SWITCH_ON:
          return {CONTROLWORD_SWITCH_ON, DriveState::SWITCHED_ON, false, true, false};
        case DriveState::SWITCH_ON_DISABLED:
          return {CONTROLWORD_SHUTDOWN, DriveState::READY_TO_SWITCH_ON, false, true, false};
        case DriveState::OPERATION_ENABLED:
          return {CONTROLWORD_SWITCH_ON, DriveState::SWITCHED_ON, false, true, false};
        case DriveState::QUICK_STOP_ACTIVE:
          return {
            CONTROLWORD_DISABLE_VOLTAGE, DriveState::SWITCH_ON_DISABLED, false, true, false};
        default:
          return {};
      }

    case DriveCommand::ENABLE_OPERATION:
      switch (current_state) {
        case DriveState::OPERATION_ENABLED:
          return {
            CONTROLWORD_ENABLE_OPERATION, DriveState::OPERATION_ENABLED, true, true, false};
        case DriveState::SWITCHED_ON:
        case DriveState::QUICK_STOP_ACTIVE:
          return {
            CONTROLWORD_ENABLE_OPERATION, DriveState::OPERATION_ENABLED, false, true, false};
        case DriveState::READY_TO_SWITCH_ON:
          return {CONTROLWORD_SWITCH_ON, DriveState::SWITCHED_ON, false, true, false};
        case DriveState::SWITCH_ON_DISABLED:
          return {CONTROLWORD_SHUTDOWN, DriveState::READY_TO_SWITCH_ON, false, true, false};
        default:
          return {};
      }

    case DriveCommand::DISABLE_OPERATION:
      switch (current_state) {
        case DriveState::OPERATION_ENABLED:
          return {CONTROLWORD_SWITCH_ON, DriveState::SWITCHED_ON, false, true, false};
        case DriveState::SWITCHED_ON:
          return {CONTROLWORD_SWITCH_ON, DriveState::SWITCHED_ON, true, true, false};
        case DriveState::READY_TO_SWITCH_ON:
          return {CONTROLWORD_SHUTDOWN, current_state, true, true, false};
        case DriveState::SWITCH_ON_DISABLED:
          return {CONTROLWORD_DISABLE_VOLTAGE, current_state, true, true, false};
        case DriveState::QUICK_STOP_ACTIVE:
          return {CONTROLWORD_QUICK_STOP, current_state, true, true, false};
        default:
          return {};
      }

    case DriveCommand::DISABLE_VOLTAGE:
      switch (current_state) {
        case DriveState::SWITCH_ON_DISABLED:
          return {
            CONTROLWORD_DISABLE_VOLTAGE, DriveState::SWITCH_ON_DISABLED, true, true, false};
        case DriveState::READY_TO_SWITCH_ON:
        case DriveState::SWITCHED_ON:
        case DriveState::OPERATION_ENABLED:
        case DriveState::QUICK_STOP_ACTIVE:
          return {
            CONTROLWORD_DISABLE_VOLTAGE, DriveState::SWITCH_ON_DISABLED, false, true, false};
        default:
          return {};
      }

    case DriveCommand::QUICK_STOP:
      switch (current_state) {
        case DriveState::QUICK_STOP_ACTIVE:
          return {CONTROLWORD_QUICK_STOP, current_state, true, true, false};
        case DriveState::SWITCH_ON_DISABLED:
          // 0x605A may make a quick stop finish in Switch on disabled.
          return {CONTROLWORD_DISABLE_VOLTAGE, current_state, true, true, false};
        case DriveState::READY_TO_SWITCH_ON:
        case DriveState::SWITCHED_ON:
        case DriveState::OPERATION_ENABLED:
          return {CONTROLWORD_QUICK_STOP, DriveState::QUICK_STOP_ACTIVE, false, true, false};
        default:
          return {};
      }

    case DriveCommand::FAULT_RESET:
      switch (current_state) {
        case DriveState::FAULT:
          return {CONTROLWORD_FAULT_RESET, DriveState::SWITCH_ON_DISABLED, false, true, false};
        case DriveState::SWITCH_ON_DISABLED:
          return {
            CONTROLWORD_DISABLE_VOLTAGE, DriveState::SWITCH_ON_DISABLED, true, true, false};
        default:
          return {};
      }
  }

  return {};
}

}  // namespace cia402_controller
