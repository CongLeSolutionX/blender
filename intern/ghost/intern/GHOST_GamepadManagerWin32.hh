/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "GHOST_GamepadManager.hh"

class GHOST_GamepadManagerWin32 : public GHOST_GamepadManager {
 private:
  /** Wait interval to check if a controller is connected (2s). */
  static constexpr uint64_t gamepad_wait_poll_interval_ = 2000;
  uint64_t gamepad_last_time_poll_;

 public:
  GHOST_GamepadManagerWin32(GHOST_System &);
  virtual ~GHOST_GamepadManagerWin32() override = default;

  virtual void send_gamepad_events(float delta_time) override final;
};
