/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Aurel Gruber
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#ifndef slim_parametrizer_h
#define slim_parametrizer_h


#include "slim_matrix_transfer.h"
#include "slim.h"
#include <igl/Timer.h>

using namespace igl;

/*	The header file that exposes the C++ functions to the native C part of Blender, see thesis.
*/

Eigen::MatrixXd getInteractiveResultBlendedWithOriginal(float blend, SLIMData *slimData);
SLIMData* setup_slim(SLIMMatrixTransfer *transferredData,
					 	 int nIterations,
						 int uvChartIndex,
						 igl::Timer &timer,
						 bool borderVerticesArePinned,
						 bool skipInitialization);
void transferUvsBackToNativePartLive(SLIMMatrixTransfer *mt,
									 Eigen::MatrixXd &UV,
									 int uvChartIndex);
void transferUvsBackToNativePart(SLIMMatrixTransfer *mt, Eigen::MatrixXd &UV, int uvChartIndex);
void param_slim_single_iteration(SLIMData *slimData);
void param_slim_live_unwrap(SLIMData *slimData,
							int n_pins,
							int* pinnedVertexIndices,
							double *pinnedVertexPositions2D,
							int n_selected_pins,
							int *selected_pins);
void param_slim(SLIMMatrixTransfer *mt, int n_iterations, bool fixBorder, bool skipInitialization);
void free_slim_data(SLIMData *slimData);
#endif // !slim_parametrizer_h
