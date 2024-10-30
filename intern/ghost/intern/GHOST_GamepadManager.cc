/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GHOST_GamepadManager.hh"
#include "GHOST_EventGamepad.hh"
#include "GHOST_WindowManager.hh"
#include <ratio>

GHOST_GamepadManager::GHOST_GamepadManager(GHOST_System &sys)
    : system_(sys), gamepad_active_(false), gamepad_state_{}, dead_zone_(0.2)
{
}

GHOST_GamepadManager::~GHOST_GamepadManager() {}

void GHOST_GamepadManager::send_button_event(GHOST_TGamepadButton button,
                                             bool press,
                                             uint64_t time,
                                             GHOST_IWindow *window)
{
  GHOST_EventGamepadButton *event = new GHOST_EventGamepadButton(time, window);
  GHOST_TEventGamepadButtonData *data = (GHOST_TEventGamepadButtonData *)event->getData();

  data->action = press ? GHOST_kPress : GHOST_kRelease;
  data->button = button;

  system_.pushEvent(event);
}

void GHOST_GamepadManager::send_gamepad_events(GHOST_GamepadState new_state, float delta_time)
{

  if (!gamepad_active_) {
    printf("A gamepad has been connected.");
  }
  gamepad_active_ = true;

  GHOST_IWindow *window = system_.getWindowManager()->getActiveWindow();

  const uint64_t now = system_.getMilliSeconds();

  const auto apply_death_zone = [this](float &val) {
    if (std::abs(val) < this->dead_zone_) {
      val = 0.0f;
    }
  };

  auto send_thumb_event =
      [&, this](float(&old_state)[2], float(&new_state)[2], GHOST_TGamepadThumb thumb) {
        apply_death_zone(new_state[0]);
        apply_death_zone(new_state[1]);
        if ((old_state[0] == 0.0f && old_state[1] == 0.0f) &&
            (new_state[0] == 0.0f && new_state[1] == 0.0f))
        {
          return;
        }
        GHOST_EventGamepadThumb *event = new GHOST_EventGamepadThumb(now, window);
        GHOST_TEventGamepadThumbData *data = (GHOST_TEventGamepadThumbData *)event->getData();
        data->value[0] = new_state[0];
        data->value[1] = new_state[1];
        data->thumb = thumb;
        data->action = new_state ? GHOST_kPress : GHOST_kRelease;
        data->dt = delta_time;
        system_.pushEvent(event);
        old_state[0] = new_state[0];
        old_state[1] = new_state[1];
      };

  send_thumb_event(gamepad_state_.left_thumb, new_state.left_thumb, GHOST_kGamepadLeftThumb);
  send_thumb_event(gamepad_state_.right_thumb, new_state.right_thumb, GHOST_kGamepadRightThumb);

  const auto send_trigger_event =
      [&, this](float &old_state, float new_state, GHOST_TGamepadTrigger trigger) {
        apply_death_zone(new_state);
        if (old_state == 0.0f && new_state == 0.0f) {
          return;
        }
        /** Send only triggers events if there is non-zero reading or the triggers are released. */
        GHOST_EventGamepadTrigger *event = new GHOST_EventGamepadTrigger(now, window);
        GHOST_TEventGamepadTriggerData *data = (GHOST_TEventGamepadTriggerData *)event->getData();
        data->value = new_state;
        data->trigger = trigger;
        data->action = new_state ? GHOST_kPress : GHOST_kRelease;
        data->dt = delta_time;
        system_.pushEvent(event);
        old_state = new_state;
      };

  send_trigger_event(
      gamepad_state_.left_trigger, new_state.left_trigger, GHOST_kGamepadLeftTrigger);
  send_trigger_event(
      gamepad_state_.right_trigger, new_state.right_trigger, GHOST_kGamepadRightTrigger);
  struct ButtonMap {
    GamepadButtonMask mask;
    GHOST_TGamepadButton event_type;
  };
  constexpr ButtonMap buttons_map[]{
      {GamepadButtonMask::A, GHOST_kGamepadButtonA},
      {GamepadButtonMask::B, GHOST_kGamepadButtonB},
      {GamepadButtonMask::X, GHOST_kGamepadButtonX},
      {GamepadButtonMask::Y, GHOST_kGamepadButtonY},

      {GamepadButtonMask::LeftShoulder, GHOST_kGamepadButtonLeftShoulder},
      {GamepadButtonMask::RightShoulder, GHOST_kGamepadButtonRightShoulder},

      {GamepadButtonMask::View, GHOST_kgamepadButtonView},
      {GamepadButtonMask::Menu, GHOST_kgamepadButtonMenu},

      {GamepadButtonMask::LeftThumb, GHOST_kGamepadButtonLeftThumb},
      {GamepadButtonMask::RightThumb, GHOST_kGamepadButtonRightThumb},

      {GamepadButtonMask::DPadUp, GHOST_kGamepadButtonDPadUp},
      {GamepadButtonMask::DPadDown, GHOST_kGamepadButtonDPadDown},
      {GamepadButtonMask::DPadLeft, GHOST_kGamepadButtonDPadLeft},
      {GamepadButtonMask::DPadRight, GHOST_kGamepadButtonDPadRight},
  };
  for (const ButtonMap &button_map : buttons_map) {
    const bool was_depressed = gamepad_state_.button_depressed[int(button_map.mask)];
    const bool is_depressed = new_state.button_depressed[int(button_map.mask)];
    if (was_depressed != is_depressed) {
      /** Send buttons events only if the button status changed. */
      send_button_event(button_map.event_type, is_depressed, now, window);
    }
  }
  gamepad_state_.button_depressed = new_state.button_depressed;
}

void GHOST_GamepadManager::set_dead_zone(const float dz)
{
  dead_zone_ = std::max(dz, 0.1f);
}

bool GHOST_GamepadManager::reset_gamepad_state()
{
  gamepad_active_ = false;
  this->gamepad_state_ = GHOST_GamepadState{};

  return true;
}

bool GHOST_GamepadManager::send_gamepad_events(float)
{
  return false;
}
