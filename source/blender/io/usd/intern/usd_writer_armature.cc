/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd_writer_armature.h"
#include "usd_armature_utils.h"
#include "usd_hierarchy_iterator.h"
#include "usd_writer_transform.h"

#include "BKE_armature.h"
#include "DNA_armature_types.h"

#include "ED_armature.hh"

#include "WM_api.hh"

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/matrix4f.h>
#include <pxr/usd/usdSkel/animation.h>
#include <pxr/usd/usdSkel/bindingAPI.h>
#include <pxr/usd/usdSkel/skeleton.h>
#include <pxr/usd/usdSkel/tokens.h>

#include <functional>
#include <iostream>

namespace usdtokens {
static const pxr::TfToken Anim("Anim", pxr::TfToken::Immortal);
}  // namespace usdtokens


static inline bool is_deform_bone(const Bone* bone) {
  return !(bone->flag & BONE_NO_DEFORM);
}

/*
 * We need a second version here because parents of deforming bones will be set on the first pass.
 */
static inline bool is_deform_bone(const Bone* bone, std::unordered_map<const char *, bool>& should_export) {
  const auto result = should_export.find(&bone->name[2]);
  if (result == should_export.end()) {
    //!TODO: Better error?
    return false;
  }

  return result->second;
}


/* Initialize the given skeleton and animation from
 * the given armature object. */
static void
initialize(const Object *obj, pxr::UsdSkelSkeleton &skel, pxr::UsdSkelAnimation &skel_anim, const bool use_deform=false) {
  using namespace blender::io::usd;

  pxr::VtTokenArray joints;
  pxr::VtArray < pxr::GfMatrix4d > bind_xforms;
  pxr::VtArray < pxr::GfMatrix4d > rest_xforms;

  std::unordered_map<const char *, const Bone*> deform_bones;
  std::unordered_map<const char *, const Bone*> deform_parent_bones;

  size_t num_bones = 0;
  size_t num_deform_bones = 0;
  size_t num_deform_parent_bones = 0;

  auto deform_visitor = [&](const Bone *bone) {
      if (!bone) {
        return;
      }

      const bool deform = is_deform_bone(bone);
      deform_bones.insert_or_assign(bone->name, bone);
      num_deform_bones += int(deform);
      num_bones += 1;
  };

  visit_bones(obj, deform_visitor);

  /* Get deform parents */
  for (auto pair : deform_bones) {
    if (pair.second) {
      Bone* parent = const_cast<Bone*>(pair.second)->parent;
      while (parent) {
        if (deform_parent_bones.find(parent->name) == deform_parent_bones.end()) {
          deform_parent_bones.insert_or_assign(parent->name, parent);
          num_deform_parent_bones += 1;
        }
        parent = parent->parent;
      }
    }
  }

  WM_reportf(RPT_WARNING, ">> [ %s ]", &obj->id.name[2]);
  WM_reportf(RPT_WARNING, "Deform bones: %d", num_deform_bones);
  WM_reportf(RPT_WARNING, "Deform parents: %d", num_deform_parent_bones);
  WM_reportf(RPT_WARNING, "Total deform:  %d", num_deform_bones + num_deform_parent_bones);
  WM_reportf(RPT_WARNING, "Total bones:  %d", num_bones);

  auto visitor = [&](const Bone *bone) {
    if (!bone) {
      return;
    }

    if (use_deform && (!is_deform_bone(bone))) {
      return;
    }

    joints.push_back(build_usd_joint_path(bone));
    const pxr::GfMatrix4f arm_mat(bone->arm_mat);
    bind_xforms.push_back(pxr::GfMatrix4d(arm_mat));

    if (bone->parent) {
      pxr::GfMatrix4f parent_arm_mat(bone->parent->arm_mat);
      const pxr::GfMatrix4f rest_mat = arm_mat * parent_arm_mat.GetInverse();
      rest_xforms.push_back(pxr::GfMatrix4d(rest_mat));
    }
    else {
      rest_xforms.push_back(pxr::GfMatrix4d(arm_mat));
    }
  };

  visit_bones(obj, visitor);
  skel.GetJointsAttr().Set(joints);
  skel.GetBindTransformsAttr().Set(bind_xforms);
  skel.GetRestTransformsAttr().Set(rest_xforms);

  pxr::UsdSkelBindingAPI usd_skel_api = pxr::UsdSkelBindingAPI::Apply(skel.GetPrim());
  usd_skel_api.CreateAnimationSourceRel().SetTargets(
      pxr::SdfPathVector({pxr::SdfPath(usdtokens::Anim)}));

  create_pose_joints(skel_anim, obj);
}

static const bPoseChannel *get_parent_pose_chan(const bPose *pose, const bPoseChannel *in_pchan)
{
  if (!(pose && in_pchan && in_pchan->bone && in_pchan->bone->parent)) {
    return nullptr;
  }

  Bone *parent = in_pchan->bone->parent;

  LISTBASE_FOREACH (const bPoseChannel *, pchan, &pose->chanbase) {
    if (pchan->bone == parent) {
      return pchan;
    }
  }

  return nullptr;
}

/* Add skeleton transform samples from the armature pose channels. */
static void add_anim_sample(pxr::UsdSkelAnimation &skel_anim,
                            const Object *obj,
                            const pxr::UsdTimeCode time)
{
  if (!(skel_anim && obj && obj->pose)) {
    return;
  }

  pxr::VtArray<pxr::GfMatrix4d> xforms;

  const bPose *pose = obj->pose;

  LISTBASE_FOREACH (const bPoseChannel *, pchan, &pose->chanbase) {

    if (!pchan->bone) {
      printf("WARNING: pchan %s is missing bone.\n", pchan->name);
      continue;
    }

    const pxr::GfMatrix4f pose_mat(pchan->pose_mat);

    if (const bPoseChannel *parent_pchan = get_parent_pose_chan(pose, pchan)) {
      const pxr::GfMatrix4f parent_pose_mat(parent_pchan->pose_mat);
      const pxr::GfMatrix4f xf = pose_mat * parent_pose_mat.GetInverse();
      xforms.push_back(pxr::GfMatrix4d(xf));
    }
    else {
      xforms.push_back(pxr::GfMatrix4d(pose_mat));
    }
  }

  skel_anim.SetTransforms(xforms, time);
}

namespace blender::io::usd {

USDArmatureWriter::USDArmatureWriter(const USDExporterContext &ctx) : USDAbstractWriter(ctx) {}

void USDArmatureWriter::do_write(HierarchyContext &context)
{
  if (!(context.object && context.object->type == OB_ARMATURE && context.object->data)) {
    BLI_assert_unreachable();
    return;
  }

  /* Create the skeleton. */
  pxr::UsdStageRefPtr stage = usd_export_context_.stage;
  pxr::UsdSkelSkeleton skel = pxr::UsdSkelSkeleton::Define(stage, usd_export_context_.usd_path);

  if (!skel) {
    WM_reportf(RPT_WARNING,
               "%s: couldn't define UsdSkelSkeleton %s\n",
               __func__,
               usd_export_context_.usd_path.GetString().c_str());
    return;
  }

  /* Create the skeleton animation primitive as a child of the skeleton. */
  pxr::SdfPath anim_path = usd_export_context_.usd_path.AppendChild(usdtokens::Anim);
  pxr::UsdSkelAnimation skel_anim = pxr::UsdSkelAnimation::Define(stage, anim_path);

  if (!skel_anim) {
    WM_reportf(RPT_WARNING,
               "%s: couldn't define UsdSkelAnimation %s\n",
               __func__,
               anim_path.GetString().c_str());
    return;
  }

  if (!this->frame_has_been_written_) {
    initialize(context.object, skel, skel_anim, 0);
  }

  add_anim_sample(skel_anim, context.object, get_export_time_code());
}

bool USDArmatureWriter::check_is_animated(const HierarchyContext &context) const
{
  const Object *obj = context.object;

  if (!(obj && obj->type == OB_ARMATURE)) {
    return false;
  }

  return obj->adt != nullptr;
}

}  // namespace blender::io::usd
