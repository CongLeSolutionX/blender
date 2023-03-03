/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2020-2022 Blender Foundation. */

#include "bvh/multi.h"
#include "device/device.h"
#include "util/foreach.h"

CCL_NAMESPACE_BEGIN

BVHMulti::BVHMulti(const BVHParams &params_,
                   const vector<Geometry *> &geometry_,
                   const vector<Object *> &objects_,
                   const Device *device_)
    : BVH(params_, geometry_, objects_), device(device_)
{
  // Resize the sub-bvh container to match the number of devices
  int n = device->get_num_devices();
  sub_bvhs.resize(n);
}

BVHMulti::~BVHMulti()
{
  foreach (BVH *bvh, sub_bvhs) {
    delete bvh;
  }
}

BVH *BVHMulti::get_device_bvh(const Device *subdevice)
{
  int id = device->device_number(subdevice);
  resize_sub_bvhs_if_needed(id);
  return sub_bvhs[id];
}

void BVHMulti::set_device_bvh(const Device *subdevice, BVH *bvh)
{
  int id = device->device_number(subdevice);
  resize_sub_bvhs_if_needed(id);
  sub_bvhs[id] = bvh;
};

/**
 * Resize the sub_bvh array if it is not big enough
 * to hold a device with the given id.
 */
void BVHMulti::resize_sub_bvhs_if_needed(int id)
{
  if ((id != -1) && (id >= sub_bvhs.size())) {
    sub_bvhs.resize(id + 1, NULL);
  }
}

void BVHMulti::replace_geometry(const vector<Geometry *> &geometry,
                                const vector<Object *> &objects)
{
  foreach (BVH *bvh, sub_bvhs) {
    bvh->replace_geometry(geometry, objects);
  }
}

CCL_NAMESPACE_END
