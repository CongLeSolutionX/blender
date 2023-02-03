/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_bvhutils.h"
#include "BKE_curves.hh"

/** \file
 * \ingroup geo
 * \brief Constraint solver for curve deformations.
 */

namespace blender::geometry {

class ConstraintSolver {
 public:
  enum SolverType {
    /* A fast single-iteration solver that solves constraints sequentially
     * from the root down (Follow-the-Leader, FTL).
     * The result is not physically correct, but very fast to compute.
     * This solver type is suitable for editing, but not so much for accurate
     * physical simulations, since the root side of a curve is infinitely stiff.
     *
     * For details see "Fast Simulation of Inextensible Hair and Fur"
     * by Matthias Mueller and Tae Yong Kim. */
    Sequential,
    /* Position Based Dynamics (PBD) solves constraints based on relative mass.
     * The solver requires multiple iterations per step. This is generally slower
     * than the FTL method, but leads to physically correct movement.
     *
     * Based on "XPBD: Position-Based Simulation of Compliant Constrained Dynamics" */
    PositionBasedDynamics,
  };

  struct Params {
    SolverType solver_type = SolverType::Sequential;

    /* Keep the distance between points constant. */
    bool use_length_constraints = true;
    /* Root point is fixed to the surface. */
    bool use_root_constraints = true;
    /* Points do not penetrate the surface. */
    bool use_collision_constraints = true;

    /* Compliance (inverse stiffness)
     * Alpha is used in physical simulation to control the softness of a constraint:
     * For alpha == 0 the constraint is stiff and the maximum correction factor is applied.
     * For values > 0 the constraint becomes squishy, and some violation is
     * permitted, and the constraint gets corrected over multiple time steps.
     */
    float alpha = 0.0f;

    /* Number of substeps to perform.
     * More substeps can be faster overall because of reduced search radius for collisions. */
    int substep_count = 20;

    /* Maximum overall distance a particle can move during a step.
     * Divide by substep count to get max substep travel.
     * This determines a the search radius for collisions.
     * A larger travel distance means the point can move faster,
     * but it can take longer to find collisions. */
    float max_travel_distance = 0.1f;

    /* Maximum number of simultaneous contacts to record per point. */
    int max_contacts_per_point = 4;

    /* Branching factor for the surface mesh BVH. */
    int bvh_branching_factor = 2;

    /* Number of iterations to satisfy constraints. */
    int max_solver_iterations = 5;

    /* Acceptable error threshold for convergence, in length units. */
    float error_threshold = 1.0e-4f;
  };

  struct Result {
    enum Status {
      Ok,
      ErrorNoConvergence,
    };

    Status status = Status::Ok;

    /* True if any point's original travel was larger than the allowed maximum.
     * If clamping is enabled the point's travel will be shorter than the input. */
    bool max_travel_exceeded = false;

    /* Total number of constraints solved. */
    int constraint_count = 0;

    /* Residual error values to judge convergence.
     * Eror computation must be explicitly enabled during solving. */
    struct {
      /* Root-mean-square of the constraint residuals, indicating numerical quality
       * of the solution. For a positional solver this value is in length units
       * and describes how far constraints are violated on average. */
      double rms_error = 0.0;
      /* Sum of squared errors (in length units). */
      double error_squared_sum = 0.0;
      /* Largest squared error. */
      double max_error_squared = 0.0;
    } residual;

    struct {
      /* Time in seconds for the entire step. */
      double step_total = 0.0;
      /* Time in seconds to build the BVH tree of the surface.
       * This is zero if the surface BVH was cached. */
      double build_bvh = 0.0;
      /* Time in seconds to find contact points, cumulative over substeps. */
      double find_contacts = 0.0;
      /* Time in seconds to solve constraints, cumulative over substeps. */
      double solve_constraints = 0.0;
    } timing;
  };

 private:
  Params params_;

  /** Length of each segment indexed by the index of the first point in the segment. */
  Array<float> segment_lengths_cu_;

  struct Contact {
    float dist_sq_;
    float3 normal_;
    float3 point_;
  };

  Array<int> contacts_num_;
  Array<Contact> contacts_;

  /** Information about the most recent step solution. */
  mutable Result result_;

 public:
  const Params &params() const;

  Span<float> segment_lengths() const;

  const Result &result() const;
  void clear_result();

  /* Initialize the solver for a given set of curves.
   * The solver must be reinitialized if the curve set changes. */
  void initialize(const Params &params, const bke::CurvesGeometry &curves, IndexMask curve_selection);

  /* Solve constraints for an independent subset of curves. */
  void step_curves(bke::CurvesGeometry &curves,
                   const Mesh *surface,
                   const bke::CurvesSurfaceTransforms &transforms,
                   Span<float3> start_positions,
                   IndexMask changed_curves,
                   bool update_error = false);

 private:
  void find_contact_points(const bke::CurvesGeometry &curves,
                           const Mesh *surface,
                           const bke::CurvesSurfaceTransforms &transforms,
                           float max_dist,
                           IndexMask changed_curves);

  void apply_distance_constraint(float3 &point_a,
                                 float3 &point_b,
                                 float segment_length,
                                 float weight_a,
                                 float weight_b) const;

  float get_distance_constraint_error(const float3 &point_a,
                                      const float3 &point_b,
                                      const float segment_length) const;

  void apply_contact_constraint(float3 &point,
                                float radius,
                                const Contact &contact) const;

  float get_contact_constraint_error(const float3 &point,
                                     float radius,
                                     const Contact &contact) const;

  void solve_constraints(bke::CurvesGeometry &curves, IndexMask changed_curves) const;

  void solve_curve_constraints(bke::CurvesGeometry &curves,
                               const VArray<float> radius,
                               const IndexRange points) const;

  void compute_error(const bke::CurvesGeometry &curves, IndexMask changed_curves) const;

  void compute_curve_error(const bke::CurvesGeometry &curves,
                           const VArray<float> radius,
                           const IndexRange points) const;
};

}  // namespace blender::geometry
