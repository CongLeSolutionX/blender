/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>
#include <mutex>

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask_expression.hh"
#include "BLI_multi_value_map.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_strict_flags.h"
#include "BLI_task.hh"
#include "BLI_timeit.hh"
#include "BLI_vector_set.hh"

namespace blender::index_mask {

constexpr int64_t inline_expr_array_size = 16;

struct FastResultSegment {
  enum class Type {
    Unknown,
    Full,
    Copy,
  };
  Type type = Type::Unknown;
  IndexRange bounds;
  const IndexMask *mask = nullptr;
};

struct FastResult {
  Vector<FastResultSegment> segments;
};

struct Boundary {
  int64_t index;
  bool is_begin;
  const FastResultSegment *segment;
};

static void sort_boundaries(MutableSpan<Boundary> boundaries)
{
  std::sort(boundaries.begin(), boundaries.end(), [](const Boundary &a, const Boundary &b) {
    return a.index < b.index;
  });
}

static FastResultSegment &evaluate_fast_make_full_segment(FastResultSegment *prev_segment,
                                                          const int64_t prev_boundary_index,
                                                          const int64_t current_boundary_index,
                                                          FastResult &result)
{
  if (prev_segment && prev_segment->type == FastResultSegment::Type::Full &&
      prev_segment->bounds.one_after_last() == prev_boundary_index)
  {
    /* Extend previous segment. */
    prev_segment->bounds = IndexRange::from_begin_end(prev_segment->bounds.first(),
                                                      current_boundary_index);
    return *prev_segment;
  }
  result.segments.append(
      {FastResultSegment::Type::Full,
       IndexRange::from_begin_end(prev_boundary_index, current_boundary_index)});
  return result.segments.last();
}

static FastResultSegment &evaluate_fast_make_unknown_segment(FastResultSegment *prev_segment,
                                                             const int64_t prev_boundary_index,
                                                             const int64_t current_boundary_index,
                                                             FastResult &result)
{
  if (prev_segment && prev_segment->type == FastResultSegment::Type::Unknown &&
      prev_segment->bounds.one_after_last() == prev_boundary_index)
  {
    /* Extend previous segment. */
    prev_segment->bounds = IndexRange::from_begin_end(prev_segment->bounds.first(),
                                                      current_boundary_index);
    return *prev_segment;
  }
  result.segments.append(
      {FastResultSegment::Type::Unknown,
       IndexRange::from_begin_end(prev_boundary_index, current_boundary_index)});
  return result.segments.last();
}

static FastResultSegment &evaluate_fast_make_copy_segment(FastResultSegment *prev_segment,
                                                          const int64_t prev_boundary_index,
                                                          const int64_t current_boundary_index,
                                                          const IndexMask &copy_from_mask,
                                                          FastResult &result)
{
  if (prev_segment && prev_segment->type == FastResultSegment::Type::Copy &&
      prev_segment->bounds.one_after_last() == prev_boundary_index &&
      prev_segment->mask == &copy_from_mask)
  {
    /* Extend previous segment. */
    prev_segment->bounds = IndexRange::from_begin_end(prev_segment->bounds.first(),
                                                      current_boundary_index);
    return *prev_segment;
  }
  result.segments.append({FastResultSegment::Type::Copy,
                          IndexRange::from_begin_end(prev_boundary_index, current_boundary_index),
                          &copy_from_mask});
  return result.segments.last();
}

BLI_NOINLINE static void evaluate_fast_union(const Span<Boundary> boundaries, FastResult &r_result)
{
  if (boundaries.is_empty()) {
    return;
  }

  FastResult &result = r_result;
  FastResultSegment *prev_segment = nullptr;
  Vector<const FastResultSegment *, 16> active_segments;
  int64_t prev_boundary_index = boundaries[0].index;

  for (const Boundary &boundary : boundaries) {
    if (prev_boundary_index < boundary.index) {
      bool has_full = false;
      bool has_unknown = false;
      bool copy_from_mask_unique = true;
      const IndexMask *copy_from_mask = nullptr;
      for (const FastResultSegment *active_segment : active_segments) {
        switch (active_segment->type) {
          case FastResultSegment::Type::Unknown: {
            has_unknown = true;
            break;
          }
          case FastResultSegment::Type::Full: {
            has_full = true;
            break;
          }
          case FastResultSegment::Type::Copy: {
            if (copy_from_mask != nullptr && copy_from_mask != active_segment->mask) {
              copy_from_mask_unique = false;
            }
            copy_from_mask = active_segment->mask;
            break;
          }
        }
      }
      if (has_full) {
        prev_segment = &evaluate_fast_make_full_segment(
            prev_segment, prev_boundary_index, boundary.index, result);
      }
      else if (has_unknown || !copy_from_mask_unique) {
        prev_segment = &evaluate_fast_make_unknown_segment(
            prev_segment, prev_boundary_index, boundary.index, result);
      }
      else if (copy_from_mask != nullptr && copy_from_mask_unique) {
        prev_segment = &evaluate_fast_make_copy_segment(
            prev_segment, prev_boundary_index, boundary.index, *copy_from_mask, result);
      }

      prev_boundary_index = boundary.index;
    }

    if (boundary.is_begin) {
      active_segments.append(boundary.segment);
    }
    else {
      active_segments.remove_first_occurrence_and_reorder(boundary.segment);
    }
  }
}

BLI_NOINLINE static void evaluate_fast_intersection(const Span<Boundary> boundaries,
                                                    const int64_t terms_num,
                                                    FastResult &r_result)
{
  if (boundaries.is_empty()) {
    return;
  }

  FastResult &result = r_result;
  FastResultSegment *prev_segment = nullptr;
  Vector<const FastResultSegment *, 16> active_segments;
  int64_t prev_boundary_index = boundaries[0].index;

  for (const Boundary &boundary : boundaries) {
    if (prev_boundary_index < boundary.index) {
      /* Only if one segment of each term is active, it's possible that the output contains
       * anything. */
      if (active_segments.size() == terms_num) {
        int full_count = 0;
        int unknown_count = 0;
        int copy_count = 0;
        bool copy_from_mask_unique = true;
        const IndexMask *copy_from_mask = nullptr;
        for (const FastResultSegment *active_segment : active_segments) {
          switch (active_segment->type) {
            case FastResultSegment::Type::Unknown: {
              unknown_count++;
              break;
            }
            case FastResultSegment::Type::Full: {
              full_count++;
              break;
            }
            case FastResultSegment::Type::Copy: {
              copy_count++;
              if (copy_from_mask != nullptr && copy_from_mask != active_segment->mask) {
                copy_from_mask_unique = false;
              }
              copy_from_mask = active_segment->mask;
              break;
            }
          }
        }
        BLI_assert(full_count + unknown_count + copy_count == terms_num);
        if (full_count == terms_num) {
          prev_segment = &evaluate_fast_make_full_segment(
              prev_segment, prev_boundary_index, boundary.index, result);
        }
        else if (unknown_count > 0 || copy_count < terms_num || !copy_from_mask_unique) {
          prev_segment = &evaluate_fast_make_unknown_segment(
              prev_segment, prev_boundary_index, boundary.index, result);
        }
        else if (copy_count == terms_num && copy_from_mask_unique) {
          prev_segment = &evaluate_fast_make_copy_segment(
              prev_segment, prev_boundary_index, boundary.index, *copy_from_mask, result);
        }
      }

      prev_boundary_index = boundary.index;
    }

    if (boundary.is_begin) {
      active_segments.append(boundary.segment);
    }
    else {
      active_segments.remove_first_occurrence_and_reorder(boundary.segment);
    }
  }
}

/* TODO: Use struct instead of pair. */
BLI_NOINLINE static void evaluate_fast_difference(const Span<std::pair<Boundary, bool>> boundaries,
                                                  FastResult &r_result)
{
  if (boundaries.is_empty()) {
    return;
  }

  FastResult &result = r_result;
  FastResultSegment *prev_segment = nullptr;
  Vector<const FastResultSegment *> active_main_segments;
  Vector<const FastResultSegment *, 16> active_subtract_segments;
  int64_t prev_boundary_index = boundaries[0].first.index;

  for (const std::pair<Boundary, bool> &boundary : boundaries) {
    if (prev_boundary_index < boundary.first.index) {
      BLI_assert(active_main_segments.size() <= 1);
      if (active_main_segments.size() == 1) {
        const FastResultSegment &active_main_segment = *active_main_segments[0];
        bool has_subtract_full = false;
        bool subtract_copy_from_mask_unique = true;
        const IndexMask *subtract_copy_from_mask = nullptr;
        for (const FastResultSegment *active_subtract_segment : active_subtract_segments) {
          switch (active_subtract_segment->type) {
            case FastResultSegment::Type::Unknown: {
              break;
            }
            case FastResultSegment::Type::Full: {
              has_subtract_full = true;
              break;
            }
            case FastResultSegment::Type::Copy: {
              if (subtract_copy_from_mask != nullptr &&
                  subtract_copy_from_mask != active_subtract_segment->mask)
              {
                subtract_copy_from_mask_unique = false;
              }
              subtract_copy_from_mask = active_subtract_segment->mask;
              break;
            }
          }
        }

        if (has_subtract_full) {
          /* Do nothing. */
        }
        else {
          switch (active_main_segment.type) {
            case FastResultSegment::Type::Unknown: {
              prev_segment = &evaluate_fast_make_unknown_segment(
                  prev_segment, prev_boundary_index, boundary.first.index, result);
              break;
            }
            case FastResultSegment::Type::Full: {
              if (active_subtract_segments.is_empty()) {
                prev_segment = &evaluate_fast_make_full_segment(
                    prev_segment, prev_boundary_index, boundary.first.index, result);
              }
              else {
                prev_segment = &evaluate_fast_make_unknown_segment(
                    prev_segment, prev_boundary_index, boundary.first.index, result);
              }
              break;
            }
            case FastResultSegment::Type::Copy: {
              if (active_subtract_segments.is_empty()) {
                prev_segment = &evaluate_fast_make_copy_segment(prev_segment,
                                                                prev_boundary_index,
                                                                boundary.first.index,
                                                                *active_main_segment.mask,
                                                                result);
              }
              else if (subtract_copy_from_mask == active_main_segment.mask &&
                       subtract_copy_from_mask_unique)
              {
                /* Do nothing. */
              }
              else {
                prev_segment = &evaluate_fast_make_unknown_segment(
                    prev_segment, prev_boundary_index, boundary.first.index, result);
              }
              break;
            }
          }
        }
      }

      prev_boundary_index = boundary.first.index;
    }

    if (boundary.second) {
      if (boundary.first.is_begin) {
        active_main_segments.append(boundary.first.segment);
      }
      else {
        active_main_segments.remove_first_occurrence_and_reorder(boundary.first.segment);
      }
    }
    else {
      if (boundary.first.is_begin) {
        active_subtract_segments.append(boundary.first.segment);
      }
      else {
        active_subtract_segments.remove_first_occurrence_and_reorder(boundary.first.segment);
      }
    }
  }
}

BLI_NOINLINE static FastResult evaluate_fast(
    const Expr &root_expression,
    const Span<const Expr *> eager_eval_order,
    const std::optional<IndexRange> eval_bounds = std::nullopt)
{
  Array<std::optional<FastResult>, inline_expr_array_size> expression_results(
      root_expression.expression_array_size());

  for (const Expr *expression : eager_eval_order) {
    FastResult &expr_result = expression_results[expression->index].emplace();
    switch (expression->type) {
      case Expr::Type::Atomic: {
        const AtomicExpr &expr = expression->as_atomic();

        IndexMask mask;
        if (eval_bounds.has_value()) {
          mask = expr.mask->slice_content(*eval_bounds);
        }
        else {
          mask = *expr.mask;
        }

        if (!mask.is_empty()) {
          const IndexRange bounds = mask.bounds();
          if (const std::optional<IndexRange> range = mask.to_range()) {
            expr_result.segments.append({FastResultSegment::Type::Full, bounds});
          }
          else {
            expr_result.segments.append({FastResultSegment::Type::Copy, bounds, expr.mask});
          }
        }
        break;
      }
      case Expr::Type::Union: {
        const UnionExpr &expr = expression->as_union();
        Vector<Boundary, 16> boundaries;
        for (const Expr *term : expr.terms) {
          const FastResult &term_result = *expression_results[term->index];
          for (const FastResultSegment &segment : term_result.segments) {
            boundaries.append({segment.bounds.first(), true, &segment});
            boundaries.append({segment.bounds.one_after_last(), false, &segment});
          }
        }
        sort_boundaries(boundaries);
        evaluate_fast_union(boundaries, expr_result);
        break;
      }
      case Expr::Type::Intersection: {
        const IntersectionExpr &expr = expression->as_intersection();
        Vector<Boundary, 16> boundaries;
        for (const Expr *term : expr.terms) {
          const FastResult &term_result = *expression_results[term->index];
          for (const FastResultSegment &segment : term_result.segments) {
            boundaries.append({segment.bounds.first(), true, &segment});
            boundaries.append({segment.bounds.one_after_last(), false, &segment});
          }
        }
        sort_boundaries(boundaries);
        evaluate_fast_intersection(boundaries, expr.terms.size(), expr_result);
        break;
      }
      case Expr::Type::Difference: {
        const DifferenceExpr &expr = expression->as_difference();
        Vector<std::pair<Boundary, bool>, 16> boundaries;
        const FastResult &main_term_result = *expression_results[expr.terms[0]->index];
        for (const FastResultSegment &segment : main_term_result.segments) {
          boundaries.append({{segment.bounds.first(), true, &segment}, true});
          boundaries.append({{segment.bounds.one_after_last(), false, &segment}, true});
        }
        for (const Expr *term : expr.terms.as_span().drop_front(1)) {
          const FastResult &term_result = *expression_results[term->index];
          for (const FastResultSegment &segment : term_result.segments) {
            boundaries.append({{segment.bounds.first(), true, &segment}, false});
            boundaries.append({{segment.bounds.one_after_last(), false, &segment}, false});
          }
        }
        std::sort(boundaries.begin(),
                  boundaries.end(),
                  [](const std::pair<Boundary, bool> &a, const std::pair<Boundary, bool> &b) {
                    return a.first.index < b.first.index;
                  });
        evaluate_fast_difference(boundaries, expr_result);
        break;
      }
    }
  }

  const FastResult &final_result = *expression_results[root_expression.index];
  return final_result;
}

struct FinalResultSegment {
  enum class Type {
    Full,
    Copy,
    Indices,
  };

  Type type = Type::Indices;
  IndexRange bounds;
  const IndexMask *copy_mask = nullptr;
  IndexMaskSegment indices;
};

BLI_NOINLINE static IndexMaskSegment evaluate_segment(const Expr &root_expression,
                                                      LinearAllocator<> &allocator,
                                                      const IndexRange bounds)
{
  BLI_assert(bounds.size() <= max_segment_size);
  const int64_t segment_offset = bounds.start();
  const int expr_array_size = root_expression.expression_array_size();
  Array<std::optional<IndexMaskSegment>, inline_expr_array_size> results(expr_array_size);
  Stack<const Expr *> expressions_to_compute;
  expressions_to_compute.push(&root_expression);

  while (!expressions_to_compute.is_empty()) {
    const Expr &expression = *expressions_to_compute.peek();
    if (results[expression.index].has_value()) {
      expressions_to_compute.pop();
      continue;
    }
    switch (expression.type) {
      case Expr::Type::Atomic: {
        const auto &expr = expression.as_atomic();
        const IndexMask sliced_mask = expr.mask->slice_content(bounds);
        const int64_t segments_num = sliced_mask.segments_num();
        IndexMaskSegment segment;
        if (segments_num == 1) {
          segment = sliced_mask.segment(0);
        }
        else if (segments_num > 1) {
          MutableSpan<int16_t> indices = allocator.allocate_array<int16_t>(sliced_mask.size());
          sliced_mask.foreach_index_optimized<int64_t>([&](const int64_t i, const int64_t pos) {
            const int64_t index = i - segment_offset;
            BLI_assert(index < max_segment_size);
            indices[pos] = int16_t(index);
          });
          segment = IndexMaskSegment(segment_offset, indices);
        }
        results[expr.index] = segment;
        break;
      }
      case Expr::Type::Union: {
        const auto &expr = expression.as_union();
        bool all_terms_computed = true;
        for (const Expr *term : expr.terms) {
          if (const std::optional<IndexMaskSegment> &term_result = results[term->index]) {
            if (term_result->size() == bounds.size()) {
              results[expr.index] = *term_result;
              break;
            }
          }
          else {
            expressions_to_compute.push(term);
            all_terms_computed = false;
            break;
          }
        }
        if (all_terms_computed && !results[expr.index]) {
          /* TODO: Generalize. */
          BLI_assert(expr.terms.size() == 2);
          const IndexMaskSegment segment_0 = *results[expr.terms[0]->index];
          const IndexMaskSegment segment_1 = *results[expr.terms[1]->index];
          const IndexMaskSegment segment_0_shift{segment_0.offset() - segment_offset,
                                                 segment_0.base_span()};
          const IndexMaskSegment segment_1_shift{segment_1.offset() - segment_offset,
                                                 segment_1.base_span()};
          Vector<int16_t, max_segment_size> indices_vec(max_segment_size);
          const int64_t indices_num = std::set_union(segment_0_shift.begin(),
                                                     segment_0_shift.end(),
                                                     segment_1_shift.begin(),
                                                     segment_1_shift.end(),
                                                     indices_vec.begin()) -
                                      indices_vec.begin();
          MutableSpan<int16_t> indices = allocator.allocate_array<int16_t>(indices_num);
          indices.copy_from(indices_vec.as_span().take_front(indices_num));
          results[expr.index] = IndexMaskSegment(segment_offset, indices);
        }
        break;
      }
      case Expr::Type::Intersection: {
        const auto &expr = expression.as_intersection();
        bool all_terms_computed = true;
        for (const Expr *term : expr.terms) {
          if (const std::optional<IndexMaskSegment> &term_result = results[term->index]) {
            if (term_result->is_empty()) {
              results[expr.index] = IndexMaskSegment();
              break;
            }
          }
          else {
            expressions_to_compute.push(term);
            all_terms_computed = false;
          }
        }
        if (all_terms_computed && !results[expr.index]) {
          /* TODO: Generalize. */
          BLI_assert(expr.terms.size() == 2);
          const IndexMaskSegment segment_0 = *results[expr.terms[0]->index];
          const IndexMaskSegment segment_1 = *results[expr.terms[1]->index];
          Vector<int64_t, max_segment_size> indices_vec(max_segment_size);
          const int64_t indices_num = std::set_intersection(segment_0.begin(),
                                                            segment_0.end(),
                                                            segment_1.begin(),
                                                            segment_1.end(),
                                                            indices_vec.begin()) -
                                      indices_vec.begin();
          MutableSpan<int16_t> indices = allocator.allocate_array<int16_t>(indices_num);
          for (const int64_t i : IndexRange(indices_num)) {
            indices[i] = int16_t(indices_vec[i] - segment_offset);
          }
          results[expr.index] = IndexMaskSegment(segment_offset, indices);
        }
        break;
      }
      case Expr::Type::Difference: {
        const auto &expr = expression.as_difference();
        bool all_terms_computed = true;
        if (const std::optional<IndexMaskSegment> main_term_result = results[expr.terms[0]->index])
        {
          if (main_term_result->is_empty()) {
            results[expr.index] = IndexMaskSegment();
            break;
          }
          for (const Expr *subtract_term : expr.terms.as_span().drop_front(1)) {
            if (const std::optional<IndexMaskSegment> subtract_term_result =
                    results[subtract_term->index])
            {
              if (subtract_term_result->size() == bounds.size()) {
                results[expr.index] = IndexMaskSegment();
                break;
              }
            }
            else {
              expressions_to_compute.push(subtract_term);
              all_terms_computed = false;
              break;
            }
          }
        }
        else {
          expressions_to_compute.push(expr.terms[0]);
          all_terms_computed = false;
          break;
        }
        if (all_terms_computed && !results[expr.index]) {
          /* TODO: Generalize. */
          BLI_assert(expr.terms.size() == 2);
          const IndexMaskSegment segment_main = *results[expr.terms[0]->index];
          const IndexMaskSegment segment_subtract = *results[expr.terms[1]->index];
          Vector<int64_t, max_segment_size> indices_vec(max_segment_size);
          const int64_t indices_num = std::set_difference(segment_main.begin(),
                                                          segment_main.end(),
                                                          segment_subtract.begin(),
                                                          segment_subtract.end(),
                                                          indices_vec.begin()) -
                                      indices_vec.begin();
          MutableSpan<int16_t> indices = allocator.allocate_array<int16_t>(indices_num);
          for (const int64_t i : IndexRange(indices_num)) {
            indices[i] = int16_t(indices_vec[i] - segment_offset);
          }
          results[expr.index] = IndexMaskSegment(segment_offset, indices);
        }
        break;
      }
    }
  }

  return *results[root_expression.index];
}

BLI_NOINLINE static Vector<IndexMaskSegment> build_result_segments(
    const Span<FinalResultSegment> final_segments)
{
  const std::array<int16_t, max_segment_size> &static_indices_array = get_static_indices_array();

  Vector<IndexMaskSegment> result_segments;
  for (const FinalResultSegment &final_segment : final_segments) {
    switch (final_segment.type) {
      case FinalResultSegment::Type::Full: {
        const int64_t full_size = final_segment.bounds.size();
        for (int64_t i = 0; i < full_size; i += max_segment_size) {
          const int64_t size = std::min(i + max_segment_size, full_size) - i;
          result_segments.append(IndexMaskSegment(final_segment.bounds.first() + i,
                                                  Span(static_indices_array).take_front(size)));
        }
        break;
      }
      case FinalResultSegment::Type::Copy: {
        const IndexMask sliced_mask = final_segment.copy_mask->slice_content(final_segment.bounds);
        sliced_mask.foreach_segment(
            [&](const IndexMaskSegment &segment) { result_segments.append(segment); });
        break;
      }
      case FinalResultSegment::Type::Indices: {
        result_segments.append(final_segment.indices);
        break;
      }
    }
  }
  return result_segments;
}

BLI_NOINLINE static Vector<const Expr *, inline_expr_array_size> compute_eager_eval_order(
    const Expr &root_expression)
{
  Vector<const Expr *, inline_expr_array_size> eval_order;
  if (root_expression.type == Expr::Type::Atomic) {
    eval_order.append(&root_expression);
    return eval_order;
  }

  Array<bool, inline_expr_array_size> is_evaluated_states(root_expression.expression_array_size(),
                                                          false);
  Stack<const Expr *, inline_expr_array_size> expr_stack;
  expr_stack.push(&root_expression);

  while (!expr_stack.is_empty()) {
    const Expr &expression = *expr_stack.peek();
    bool &is_evaluated = is_evaluated_states[expression.index];
    if (is_evaluated) {
      expr_stack.pop();
      continue;
    }
    bool all_terms_evaluated = true;
    for (const Expr *term : expression.terms) {
      bool &term_evaluated = is_evaluated_states[term->index];
      if (!term_evaluated) {
        if (term->type == Expr::Type::Atomic) {
          eval_order.append(term);
          term_evaluated = true;
        }
        else {
          expr_stack.push(term);
          all_terms_evaluated = false;
        }
      }
    }
    if (all_terms_evaluated) {
      eval_order.append(&expression);
      is_evaluated = true;
      expr_stack.pop();
    }
  }

  return eval_order;
}

BLI_NOINLINE static IndexMask evaluate_expression_impl(const Expr &root_expression,
                                                       IndexMaskMemory &memory)
{
  Vector<FinalResultSegment, 16> final_segments;
  Stack<IndexRange, 16> long_unknown_segments;
  Vector<IndexRange, 16> short_unknown_segments;

  const Vector<const Expr *, inline_expr_array_size> eager_eval_order = compute_eager_eval_order(
      root_expression);

  auto handle_fast_result = [&](const FastResult &fast_result) {
    for (const FastResultSegment &segment : fast_result.segments) {
      switch (segment.type) {
        case FastResultSegment::Type::Unknown: {
          if (segment.bounds.size() > max_segment_size) {
            long_unknown_segments.push(segment.bounds);
          }
          else {
            short_unknown_segments.append(segment.bounds);
          }
          break;
        }
        case FastResultSegment::Type::Copy: {
          BLI_assert(segment.mask);
          final_segments.append({FinalResultSegment::Type::Copy, segment.bounds, segment.mask});
          break;
        }
        case FastResultSegment::Type::Full: {
          final_segments.append({FinalResultSegment::Type::Full, segment.bounds});
          break;
        }
      }
    }
  };
  const FastResult initial_fast_result = evaluate_fast(root_expression, eager_eval_order);
  handle_fast_result(initial_fast_result);

  while (!long_unknown_segments.is_empty()) {
    const IndexRange unknown_bounds = long_unknown_segments.pop();
    const int64_t split_pos = unknown_bounds.size() / 2;
    const IndexRange left_half = unknown_bounds.take_front(split_pos);
    const IndexRange right_half = unknown_bounds.drop_front(split_pos);
    const FastResult left_result = evaluate_fast(root_expression, eager_eval_order, left_half);
    const FastResult right_result = evaluate_fast(root_expression, eager_eval_order, right_half);
    handle_fast_result(left_result);
    handle_fast_result(right_result);
  }

  auto evaluate_unknown_segment = [&](const IndexRange bounds,
                                      LinearAllocator<> &allocator,
                                      Vector<FinalResultSegment, 16> &segments) {
    const IndexMaskSegment indices = evaluate_segment(root_expression, allocator, bounds);
    if (!indices.is_empty()) {
      segments.append({FinalResultSegment::Type::Indices, bounds, nullptr, indices});
    }
  };

  const int64_t unknown_segment_eval_grain_size = 8;
  if (short_unknown_segments.size() < unknown_segment_eval_grain_size) {
    for (const IndexRange &bounds : short_unknown_segments) {
      evaluate_unknown_segment(bounds, memory, final_segments);
    }
  }
  else {
    struct LocalData {
      LinearAllocator<> allocator;
      Vector<FinalResultSegment, 16> segments;
    };
    threading::EnumerableThreadSpecific<LocalData> data_by_thread;
    threading::parallel_for(short_unknown_segments.index_range(),
                            unknown_segment_eval_grain_size,
                            [&](const IndexRange range) {
                              LocalData &data = data_by_thread.local();
                              for (const IndexRange &bounds :
                                   short_unknown_segments.as_span().slice(range)) {
                                evaluate_unknown_segment(bounds, data.allocator, data.segments);
                              }
                            });
    for (LocalData &data : data_by_thread) {
      if (!data.segments.is_empty()) {
        final_segments.extend(data.segments);
        memory.transfer_ownership_from(data.allocator);
      }
    }
  }

  if (final_segments.is_empty()) {
    return {};
  }
  if (final_segments.size() == 1) {
    const FinalResultSegment &final_segment = final_segments[0];
    switch (final_segment.type) {
      case FinalResultSegment::Type::Full: {
        return IndexMask(IndexRange(final_segment.bounds));
      }
      case FinalResultSegment::Type::Copy: {
        return final_segment.copy_mask->slice_content(final_segment.bounds);
      }
      case FinalResultSegment::Type::Indices: {
        return IndexMask::from_segments({final_segment.indices}, memory);
      }
    }
  }

  std::sort(final_segments.begin(),
            final_segments.end(),
            [](const FinalResultSegment &a, const FinalResultSegment &b) {
              return a.bounds.start() < b.bounds.start();
            });

  Vector<IndexMaskSegment> result_segments = build_result_segments(final_segments);
  return IndexMask::from_segments(result_segments, memory);
}

IndexMask evaluate_expression(const Expr &expression, IndexMaskMemory &memory)
{
  return evaluate_expression_impl(expression, memory);
}

}  // namespace blender::index_mask
