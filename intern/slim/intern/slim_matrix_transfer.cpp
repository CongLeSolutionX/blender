/* SPDX-FileCopyrightText: 2023 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <Eigen/Dense>

#include "slim.h"
#include "slim_matrix_transfer.h"
#include "geometry_data_retrieval.h"

namespace slim {

MatrixTransferChart::MatrixTransferChart() = default;
MatrixTransferChart::MatrixTransferChart(MatrixTransferChart &&) = default;
MatrixTransferChart::~MatrixTransferChart() = default;
MatrixTransfer::MatrixTransfer() = default;
MatrixTransfer::~MatrixTransfer() = default;

void MatrixTransferChart::free_slim_data()
{
  data.reset(nullptr);
}

/* Transfers all the matrices from the native part and initialises SLIM. */
void MatrixTransfer::setup_slim_data(MatrixTransferChart &chart, int n_iterations) const
{
  SLIMDataPtr slim_data = std::make_unique<SLIMDataPtr::element_type>();

  try {
    if (!chart.succeeded) {
      throw SlimFailedException();
    }

    GeometryData geometry_data(*this, chart);
    geometry_data.construct_slim_data(
        *slim_data, skip_initialization, reflection_mode, n_iterations);

    chart.n_pinned_vertices = geometry_data.number_of_pinned_vertices;
  }
  catch (SlimFailedException &) {
    slim_data->valid = false;
    chart.succeeded = false;
  }

  chart.data = std::move(slim_data);
}

}  // namespace slim