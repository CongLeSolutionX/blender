/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 */

#include "ANIM_action.hh"
#include "ANIM_animdata.hh"
#include "ANIM_asset.hh"
#include "ANIM_keyframing.hh"

#include "BLI_assert.h"

struct Main;

namespace blender::animrig::asset {

static const Slot *asset_slot_for_id(const ID &animated_id, const Action &action_asset)
{
  const Slot *first_suitable_slot = nullptr;
  for (const Slot *slot : action_asset.slots()) {
    if (!slot->is_suitable_for(animated_id)) {
      continue;
    }

    /* Ignore slots without animation. */
    const ChannelBag *cbag = channelbag_for_action_slot(action_asset, slot->handle);
    if (!cbag || cbag->fcurves().is_empty()) {
      continue;
    }

    /* This is a usable slot. Unless there is a slot that also matches the name
     * perfectly, use this one. */
    if (!first_suitable_slot) {
      first_suitable_slot = slot;
    }

    /* Do not compare the first two characters, as they can be XX for an unspecified ID type. */
    if (slot->name_without_prefix() == animated_id.name + 2) {
      return slot;
    }
  }
  return first_suitable_slot;
}

InjectActionResult inject_action(Main &bmain,
                                 ID &animated_id,
                                 const Action &action_asset,
                                 const float frame)
{
  /* Find a slot to take keyframes from. */
  const Slot *asset_slot = asset_slot_for_id(animated_id, action_asset);
  if (!asset_slot) {
    return InjectActionResult::NO_SUITABLE_SLOT;
  }

  const ChannelBag *asset_channelbag = channelbag_for_action_slot(action_asset,
                                                                  asset_slot->handle);
  BLI_assert(asset_channelbag);

  /* Find the asset's start frame, to compute the correct offset for copying keys. */
  const float2 asset_start_end_frame = action_asset.get_frame_range_of_slot(asset_slot->handle);

  /* Prepare the animated ID so it is sure to have an Action that's keyable. */
  bAction *dna_action = id_action_ensure(&bmain, &animated_id);
  BLI_assert(dna_action != nullptr);
  Action &target_action = dna_action->wrap();
  auto [target_layer, target_slot] = prep_action_layer_for_keying(target_action, animated_id);
  BLI_assert(target_layer);
  BLI_assert(target_slot);

  Strip *target_strip = target_layer->strip(0);
  StripKeyframeData &target_strip_data = target_strip->data<StripKeyframeData>(target_action);
  ChannelBag &target_cbag = target_strip_data.channelbag_for_slot_ensure(*target_slot);

  /* TODO: Ensure that the right Channel Groups exist. */

  /* Go over the F-Curves and inject their curves into the target Action. */
  const float frame_offset = frame - asset_start_end_frame[0];
  const KeyframeSettings settings = get_keyframe_settings(false);

  for (const FCurve *asset_fcurve : asset_channelbag->fcurves()) {
    FCurve &target_fcurve = target_cbag.fcurve_ensure(
        &bmain, {asset_fcurve->rna_path, asset_fcurve->array_index});

    foreach_fcurve_key(
        const_cast<FCurve *>(asset_fcurve), [&](FCurve &, int, const BezTriple &asset_bezt) {
          insert_vert_fcurve(&target_fcurve,
                             {asset_bezt.vec[1][0] + frame_offset, asset_bezt.vec[1][1]},
                             settings,
                             INSERTKEY_OVERWRITE_FULL);
          return true;
        });
  }

  return InjectActionResult::OK;
}

}  // namespace blender::animrig::asset
