/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "transform.hh"
#include "transform_convert.hh"

using namespace blender;

static void custom_create_trans_data(bContext * /*C*/, TransInfo *t)
{
  /* Created already. */
  UNUSED_VARS_NDEBUG(t);
  BLI_assert(TRANS_DATA_CONTAINER_FIRST_SINGLE(t));
}

/* -------------------------------------------------------------------- */
/** \name Recalc Data
 * \{ */

static void custom_recalc_data(TransInfo *t)
{
  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    TcReservedData *tc_custom_data = static_cast<TcReservedData *>(tc->custom.type.data);
    tc_custom_data->recalc_data_fn(tc_custom_data->userdata);
  }
}

/** \} */

static void custom_special_aftertrans_update(bContext * /*C*/, TransInfo *t)
{
  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    TcReservedData *tc_custom_data = static_cast<TcReservedData *>(tc->custom.type.data);
    tc_custom_data->finish_fn(tc_custom_data->userdata, t->state == TRANS_CANCEL);
  }
}

TransConvertTypeInfo TransConvertType_Custom = {
    /*flags*/ (T_EDIT | T_POINTS),
    /*create_trans_data*/ custom_create_trans_data,
    /*recalc_data*/ custom_recalc_data,
    /*special_aftertrans_update*/ custom_special_aftertrans_update,
};
