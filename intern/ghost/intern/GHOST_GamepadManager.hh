/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once
#ifndef WITH_INPUT_GAMEPAD
#  error Gamepad code included in non-Gamepad-enabled build
#endif

#include <bitset>

#include "GHOST_System.hh"
enum class GamepadButtonMask {
  A = 0,
  B,
  X,
  Y,

  LeftShoulder,
  RightShoulder,

  View,
  Menu,

  LeftThumb,
  RightThumb,

  DPadUp,
  DPadDown,
  DPadLeft,
  DPadRight,
};

struct GHOST_GamepadState {
  float left_thumb[2] = {0.0f};
  float right_thumb[2] = {0.0f};

  float left_trigger = 0.0f;
  float right_trigger = 0.0f;

  std::bitset<14> button_depressed = false;
};

class GHOST_GamepadManager {
 public:
  GHOST_GamepadManager(GHOST_System &);
  virtual ~GHOST_GamepadManager();

  void set_dead_zone(const float);

 protected:
  /** Reset the current gamepad status, used if a gamepad is not longer available. */
  bool reset_gamepad_state();

 public:
  virtual bool send_gamepad_events(float delta_time);

 protected:
  /** Active gamepad input snapshot. */

  /** Update the current gamepad status, and sends events for buttons that status changed. */
 protected:
  void send_gamepad_events(GHOST_GamepadState new_state, float delta_time);

  /** Send button events. */
  void send_button_event(GHOST_TGamepadButton button,
                         bool press,
                         uint64_t time,
                         GHOST_IWindow *window);

  GHOST_System &system_;
  bool gamepad_active_;
  /** Gamepad snapshot. */
  GHOST_GamepadState gamepad_state_;
  float dead_zone_;
};
