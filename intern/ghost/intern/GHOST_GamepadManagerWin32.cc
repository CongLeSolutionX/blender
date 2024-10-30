/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GHOST_GamepadManagerWin32.hh"

#include <limits>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#pragma comment(lib, "xinput")
#include <xinput.h>

GHOST_GamepadManagerWin32::GHOST_GamepadManagerWin32(GHOST_System &sys)
    : GHOST_GamepadManager(sys), gamepad_last_time_poll_(sys.getMilliSeconds())
{
}

GHOST_GamepadManagerWin32::~GHOST_GamepadManagerWin32() {}

bool GHOST_GamepadManagerWin32::send_gamepad_events(float delta_time)
{
  const uint64_t time = system_.getMilliSeconds();

  if (!this->gamepad_active_) {
    if (time - this->gamepad_last_time_poll_ < this->gamepad_poll_interval_) {
      return false;
    }
  }
  this->gamepad_last_time_poll_ = time;

  XINPUT_STATE input_state{0};
  DWORD dwResult = XInputGetState(0, &input_state);

  if (dwResult == ERROR_SUCCESS) {
    XINPUT_GAMEPAD &gamepad = input_state.Gamepad;

    constexpr float shrt_max_float = float(std::numeric_limits<short>::max());
    constexpr float uchar_max_float = float(std::numeric_limits<unsigned char>::max());

    GHOST_GamepadState new_state = {};

    new_state.left_thumb[0] = float(gamepad.sThumbLX) / shrt_max_float;
    new_state.left_thumb[1] = float(gamepad.sThumbLY) / shrt_max_float;
    new_state.right_thumb[0] = float(gamepad.sThumbRX) / shrt_max_float;
    new_state.right_thumb[1] = float(gamepad.sThumbRY) / shrt_max_float;
    new_state.left_trigger = float(gamepad.bLeftTrigger) / uchar_max_float;
    new_state.right_trigger = float(gamepad.bRightTrigger) / uchar_max_float;

    struct ButtonMap {
      GamepadButtonMask dst;
      int src;
    };
    constexpr ButtonMap buttons_map[]{
        {GamepadButtonMask::A, XINPUT_GAMEPAD_A},
        {GamepadButtonMask::B, XINPUT_GAMEPAD_B},
        {GamepadButtonMask::X, XINPUT_GAMEPAD_X},
        {GamepadButtonMask::Y, XINPUT_GAMEPAD_Y},

        {GamepadButtonMask::LeftShoulder, XINPUT_GAMEPAD_LEFT_SHOULDER},
        {GamepadButtonMask::RightShoulder, XINPUT_GAMEPAD_RIGHT_SHOULDER},

        {GamepadButtonMask::View, XINPUT_GAMEPAD_BACK},
        {GamepadButtonMask::Menu, XINPUT_GAMEPAD_START},

        {GamepadButtonMask::LeftThumb, XINPUT_GAMEPAD_LEFT_THUMB},
        {GamepadButtonMask::RightThumb, XINPUT_GAMEPAD_RIGHT_THUMB},

        {GamepadButtonMask::DPadUp, XINPUT_GAMEPAD_DPAD_UP},
        {GamepadButtonMask::DPadDown, XINPUT_GAMEPAD_DPAD_DOWN},
        {GamepadButtonMask::DPadLeft, XINPUT_GAMEPAD_DPAD_LEFT},
        {GamepadButtonMask::DPadRight, XINPUT_GAMEPAD_DPAD_RIGHT},
    };

    for (const ButtonMap &button_map : buttons_map) {
      new_state.button_depressed[int(button_map.dst)] = bool(gamepad.wButtons & button_map.src);
    }

    GHOST_GamepadManager::send_gamepad_events(new_state, delta_time);
    return true;
  }
  else {
    GHOST_GamepadManager::reset_gamepad_state();
    return false;
  }
}
