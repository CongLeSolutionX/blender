/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "GHOST_GamepadManager.hh"

class GHOST_GamepadManagerWin32 : public GHOST_GamepadManager {
 private:
  /** 2s.*/
  static constexpr uint64_t gamepad_poll_interval_ = 2000;
  uint64_t gamepad_last_time_poll_ = 2000;

 public:
  GHOST_GamepadManagerWin32(GHOST_System &);
  ~GHOST_GamepadManagerWin32();

  /** Retrieves the current state of the gamepad reported by the system. */
  virtual bool send_gamepad_events(float delta_time) override final;
};
