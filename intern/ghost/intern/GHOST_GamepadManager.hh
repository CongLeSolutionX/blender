/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifndef WITH_INPUT_GAMEPAD
#  error Gamepad code included in non-Gamepad-enabled build
#endif

#include "GHOST_System.hh"

#include <bitset>

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
  virtual ~GHOST_GamepadManager() = default;
  void set_dead_zone(const float);

  /** Once per frame checks any change in the gamepad state and send events. */
  virtual void send_gamepad_events(float delta_time) = 0;

 protected:
  /**
   * Reset the current gamepad state, used when a gamepad is not longer available.
   * This will trigger release button events if any button was being hold.
   */
  void reset_gamepad_state();
  /**
   * Generate gamepad events by checking any change in the stored gamepad snapshot against the new
   * gamepad state. Also updates the gamepad snapshot.
   */
  void send_gamepad_events(GHOST_GamepadState new_state, float delta_time);

  GHOST_System &system_;
  bool gamepad_active_;
  /** Gamepad snapshot. */
  GHOST_GamepadState gamepad_state_;
  float dead_zone_;
};
