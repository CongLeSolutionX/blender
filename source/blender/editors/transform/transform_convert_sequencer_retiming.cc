/* SPDX-FileCopyrightText: 2021 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "MEM_guardedalloc.h"

#include "DNA_space_types.h"

#include "BLI_listbase.h"
#include "BLI_math.h"

#include "BKE_context.h"
#include "BKE_report.h"

#include "SEQ_relations.h"
#include "SEQ_retiming.h"
#include "SEQ_retiming.hh"
#include "SEQ_sequencer.h"
#include "SEQ_time.h"

#include "ED_keyframing.h"

#include "UI_view2d.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

#include "transform.h"
#include "transform_convert.h"

/** Used for sequencer transform. */
typedef struct TransDataSeq {
  Sequence *seq;
  int orig_timeline_frame;
  int handle_index; /* Some actions may need to destroy original data, use index to access it. */
} TransDataSeq;

static TransData *SeqToTransData(const Scene *scene,
                                 Sequence *seq,
                                 SeqRetimingHandle *handle,
                                 TransData *td,
                                 TransData2D *td2d,
                                 TransDataSeq *tdseq)
{

  td2d->loc[0] = SEQ_retiming_handle_timeline_frame_get(scene, seq, handle);
  td2d->loc[1] = handle->retiming_factor;
  td2d->loc2d = NULL;
  td->loc = td2d->loc;
  copy_v3_v3(td->iloc, td->loc);
  copy_v3_v3(td->center, td->loc);
  memset(td->axismtx, 0, sizeof(td->axismtx));
  td->axismtx[2][2] = 1.0f;
  unit_m3(td->mtx);
  unit_m3(td->smtx);

  tdseq->seq = seq;
  tdseq->orig_timeline_frame = SEQ_retiming_handle_timeline_frame_get(scene, seq, handle);
  tdseq->handle_index = SEQ_retiming_handle_index_get(seq, handle);

  td->extra = (void *)tdseq;
  td->ext = NULL;
  td->flag |= TD_SELECTED;
  td->dist = 0.0;

  return td;
}

static void freeSeqData(TransInfo * /*t*/,
                        TransDataContainer *tc,
                        TransCustomData * /*custom_data*/)
{
  TransData *td = (TransData *)tc->data;
  MEM_freeN(td->extra);
}

static void createTransSeqRetimingData(bContext * /*C*/, TransInfo *t)
{
  if (SEQ_editing_get(t->scene) == NULL) {
    return;
  }

  blender::Vector selection = SEQ_retiming_selection_get(t->scene);

  if (selection.size() == 0) {
    return;
  }

  TransDataContainer *tc = TRANS_DATA_CONTAINER_FIRST_SINGLE(t);
  tc->custom.type.free_cb = freeSeqData;

  tc->data_len = selection.size();
  tc->data = MEM_cnew_array<TransData>(tc->data_len, "TransSeq TransData");
  tc->data_2d = MEM_cnew_array<TransData2D>(tc->data_len, "TransSeq TransData2D");
  TransDataSeq *tdseq = MEM_cnew_array<TransDataSeq>(tc->data_len, "TransSeq TransDataSeq");
  TransData *td = tc->data;
  TransData2D *td2d = tc->data_2d;

  /*for elem*/
  for (const RetimingSelectionElem *elem : selection) {
    SeqToTransData(t->scene, elem->seq, elem->handle, td++, td2d++, tdseq++);
  }
}

static void recalcData_sequencer_retiming(TransInfo *t)
{
  TransDataContainer *tc = TRANS_DATA_CONTAINER_FIRST_SINGLE(t);
  TransData *td = NULL;
  TransData2D *td2d = NULL;
  int i;

  for (i = 0, td = tc->data, td2d = tc->data_2d; i < tc->data_len; i++, td++, td2d++) {
    TransDataSeq *tdseq = static_cast<TransDataSeq *>(td->extra);
    Sequence *seq = tdseq->seq;

    /* Calculate translation. */
    float translation = td2d->loc[0] - tdseq->orig_timeline_frame;

    /* Apply translation. */
    blender::MutableSpan handles = SEQ_retiming_handles_get(seq);
    SeqRetimingHandle *handle = &handles[tdseq->handle_index];
    SEQ_retiming_handle_timeline_frame_set(t->scene, seq, handle, td2d->loc[0]);

    SEQ_relations_invalidate_cache_preprocessed(t->scene, seq);
  }
}

static void special_aftertrans_update__sequencer_retiming(bContext * /*C*/, TransInfo *t)
{

  TransDataContainer *tc = TRANS_DATA_CONTAINER_FIRST_SINGLE(t);
  TransData *td = NULL;
  TransData2D *td2d = NULL;
  int i;

  for (i = 0, td = tc->data, td2d = tc->data_2d; i < tc->data_len; i++, td++, td2d++) {
    TransDataSeq *tdseq = static_cast<TransDataSeq *>(td->extra);
    Sequence *seq = tdseq->seq;
    if (t->state == TRANS_CANCEL) {
      // ??????
    }
  }
}

TransConvertTypeInfo TransConvertType_SequencerRetiming = {
    /*flags*/ (T_POINTS | T_2D_EDIT),
    /*createTransData*/ createTransSeqRetimingData,
    /*recalcData*/ recalcData_sequencer_retiming,
    /*special_aftertrans_update*/ special_aftertrans_update__sequencer_retiming,
};
