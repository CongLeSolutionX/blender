/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup GHOST
 */

#pragma once

#ifndef WITH_INPUT_GAMEPAD
#  error Gamepad code included in non-gamepad-enabled build
#endif

#include "GHOST_Event.hh"

/**
 * Gamepad trigger event.
 * Events that contains input reading from gamepad triggers, which generates 1d analog data.
 * These events are only sent once every frame.
 */
class GHOST_EventGamepadTrigger : public GHOST_Event {
 protected:
  GHOST_TEventGamepadTriggerData trigger_data_;

 public:
  GHOST_EventGamepadTrigger(uint64_t time, GHOST_IWindow *window)
      : GHOST_Event(time, GHOST_kEventGamepadTrigger, window), trigger_data_{}
  {
    m_data = &trigger_data_;
  }
};

/**
 * Gamepad thumbstick event.
 * Events that contains input reading from gamepad thumbsticks, which generates 2d analog data.
 * These events are only sent once every frame.
 */
class GHOST_EventGamepadThumb : public GHOST_Event {
 protected:
  GHOST_TEventGamepadThumbData thumb_data_;

 public:
  GHOST_EventGamepadThumb(uint64_t time, GHOST_IWindow *window)
      : GHOST_Event(time, GHOST_kEventGamepadThumb, window), thumb_data_{}
  {
    m_data = &thumb_data_;
  }
};

/**
 * Gamepad button event.
 */
class GHOST_EventGamepadButton : public GHOST_Event {
 protected:
  GHOST_TEventGamepadButtonData button_data_;

 public:
  GHOST_EventGamepadButton(uint64_t time, GHOST_IWindow *window)
      : GHOST_Event(time, GHOST_kEventGamepadButton, window), button_data_{}
  {
    m_data = &button_data_;
  }
};
