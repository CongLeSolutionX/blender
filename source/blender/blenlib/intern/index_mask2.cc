/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_bit_vector.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask2.hh"
#include "BLI_set.hh"
#include "BLI_strict_flags.h"
#include "BLI_task.hh"
#include "BLI_timeit.hh"

namespace blender::index_mask {

std::array<int16_t, chunk_capacity> build_static_indices_array()
{
  std::array<int16_t, chunk_capacity> data;
  for (int16_t i = 0; i < chunk_capacity; i++) {
    data[size_t(i)] = i;
  }
  return data;
}

const IndexMask &get_static_index_mask_for_min_size(const int64_t min_size)
{
  static constexpr int64_t size_shift = 30;
  static constexpr int64_t max_size = (1 << size_shift);
  static constexpr int64_t chunks_num = max_size / chunk_capacity;

  BLI_assert(min_size <= max_size);
  UNUSED_VARS_NDEBUG(min_size);

  static IndexMask static_mask = []() {
    static Array<Chunk> chunks_array(chunks_num);
    static Array<int64_t> chunk_ids_array(chunks_num);
    static Array<int64_t> cumulative_chunk_sizes(chunks_num + 1);

    static const int16_t *static_offsets = get_static_indices_array().data();
    static const int16_t static_cumulative_segment_sizes[2] = {0, chunk_capacity};

    threading::parallel_for(IndexRange(chunks_num), 1024, [&](const IndexRange range) {
      for (const int64_t i : range) {
        Chunk &chunk = chunks_array[i];
        chunk.segments_num = 1;
        chunk.indices_by_segment = &static_offsets;
        chunk.cumulative_segment_sizes = static_cumulative_segment_sizes;

        chunk_ids_array[i] = i;
        cumulative_chunk_sizes[i] = i * chunk_capacity;
      }
    });
    cumulative_chunk_sizes.last() = max_size;

    IndexMask mask;
    IndexMaskData &mask_data = mask.data_for_inplace_construction();
    mask_data.chunks_num = chunks_num;
    mask_data.indices_num = max_size;
    mask_data.chunks = chunks_array.data();
    mask_data.chunk_ids = chunk_ids_array.data();
    mask_data.cumulative_chunk_sizes = cumulative_chunk_sizes.data();
    mask_data.begin_it.segment_i = 0;
    mask_data.begin_it.index_in_segment = 0;
    mask_data.end_it.segment_i = 0;
    mask_data.end_it.index_in_segment = chunk_capacity;
    return mask;
  }();
  return static_mask;
}

std::ostream &operator<<(std::ostream &stream, const IndexMask &mask)
{
  Array<int64_t> indices(mask.size());
  mask.to_indices<int64_t>(indices);
  Vector<std::variant<IndexRange, Span<int64_t>>> segments;
  unique_sorted_indices::split_to_ranges_and_spans<int64_t>(indices, 16, segments);
  std::cout << "(Size: " << mask.size() << " | ";
  for (const std::variant<IndexRange, Span<int64_t>> &segment : segments) {
    if (std::holds_alternative<IndexRange>(segment)) {
      const IndexRange range = std::get<IndexRange>(segment);
      std::cout << range;
    }
    else {
      const Span<int64_t> segment_indices = std::get<Span<int64_t>>(segment);
      std::cout << "[";
      for (const int64_t index : segment_indices) {
        std::cout << index << ",";
      }
      std::cout << "]";
    }
    std::cout << ", ";
  }
  std::cout << ")";
  return stream;
}

namespace unique_sorted_indices {

template<typename T>
static void split_by_chunk_recursive(const Span<T> indices,
                                     const int64_t offset,
                                     Vector<IndexRange> &r_chunks)
{
  if (indices.is_empty()) {
    return;
  }
  const T first_index = indices.first();
  const T last_index = indices.last();
  const int64_t first_chunk_id = index_to_chunk_id(first_index);
  const int64_t last_chunk_id = index_to_chunk_id(last_index);
  if (first_chunk_id == last_chunk_id) {
    r_chunks.append_as(offset, indices.size());
    return;
  }
  const int64_t middle_chunk_index = (first_chunk_id + last_chunk_id + 1) / 2;
  const int64_t split_value = middle_chunk_index * chunk_capacity - 1;
  const int64_t left_split_size = std::upper_bound(indices.begin(), indices.end(), split_value) -
                                  indices.begin();
  split_by_chunk_recursive(indices.take_front(left_split_size), offset, r_chunks);
  split_by_chunk_recursive(
      indices.drop_front(left_split_size), offset + left_split_size, r_chunks);
}

template<typename T> Vector<IndexRange> split_by_chunk(const Span<T> indices)
{
  BLI_assert(std::is_sorted(indices.begin(), indices.end()));
  Vector<IndexRange> chunks;
  /* This can be too low in some cases, but it's never too large. */
  chunks.reserve(size_to_chunk_num(indices.size()));
  split_by_chunk_recursive(indices, 0, chunks);
  return chunks;
}

template<typename T>
int64_t split_to_ranges_and_spans(const Span<T> indices,
                                  const int64_t range_threshold,
                                  Vector<std::variant<IndexRange, Span<T>>> &r_parts)
{
  BLI_assert(range_threshold >= 1);
  const int64_t old_parts_num = r_parts.size();
  Span<T> remaining_indices = indices;
  while (!remaining_indices.is_empty()) {
    if (non_empty_is_range(remaining_indices)) {
      /* All remaining indices are range. */
      r_parts.append(non_empty_as_range(remaining_indices));
      break;
    }
    if (non_empty_is_range(remaining_indices.take_front(range_threshold))) {
      /* Next segment is a range. Now find the place where the range ends. */
      const int64_t segment_size = find_size_of_next_range(remaining_indices);
      r_parts.append(IndexRange(remaining_indices[0], segment_size));
      remaining_indices = remaining_indices.drop_front(segment_size);
      continue;
    }
    /* Next segment is just indices. Now find the place where the next range starts. */
    const int64_t segment_size = find_size_until_next_range(remaining_indices, range_threshold);
    r_parts.append(remaining_indices.take_front(segment_size));
    remaining_indices = remaining_indices.drop_front(segment_size);
  }
  return r_parts.size() - old_parts_num;
}

template<typename T> IndexMask to_index_mask(const Span<T> indices, LinearAllocator<> &allocator)
{
  if (indices.is_empty()) {
    return {};
  }
  if (non_empty_is_range(indices)) {
    return non_empty_as_range(indices);
  }
  const Vector<IndexRange> split_ranges = split_by_chunk(indices);
  const int64_t chunks_num = split_ranges.size();

  MutableSpan<Chunk> chunks = allocator.allocate_array<Chunk>(chunks_num);
  MutableSpan<int64_t> chunk_ids = allocator.allocate_array<int64_t>(chunks_num);

  static const int16_t *static_offsets = get_static_indices_array().data();

  /* TODO: Use that again to avoid some allocations. */
  [[maybe_unused]] const Chunk full_chunk_template = IndexMask(chunk_capacity).data().chunks[0];

  std::mutex scope_mutex;

  threading::parallel_for(split_ranges.index_range(), 32, [&](const IndexRange slice) {
    Vector<std::variant<IndexRange, Span<T>>> segments_in_chunks;
    Vector<int64_t> segments_per_chunk_cumulative;
    segments_per_chunk_cumulative.reserve(slice.size() + 1);
    segments_per_chunk_cumulative.append(0);
    int64_t index_allocations_num = 0;
    Vector<int64_t> chunks_to_postprocess;

    for (const int64_t index_in_slice : IndexRange(slice.size())) {
      const int64_t chunk_i = slice[index_in_slice];
      const IndexRange range_for_chunk = split_ranges[chunk_i];
      const Span<T> indices_in_chunk = indices.slice(range_for_chunk);
      BLI_assert(!indices_in_chunk.is_empty());

      const int64_t chunk_id = index_to_chunk_id(int64_t(indices_in_chunk[0]));
      BLI_assert(chunk_id == index_to_chunk_id(int64_t(indices_in_chunk.last())));
      Chunk &chunk = chunks[chunk_i];
      chunk_ids[chunk_i] = chunk_id;

      if (indices_in_chunk.size() == chunk_capacity) {
        chunk = full_chunk_template;
        continue;
      }
      chunks_to_postprocess.append(index_in_slice);

      const int16_t segments_in_chunk_num = int16_t(
          split_to_ranges_and_spans(indices_in_chunk, 16, segments_in_chunks));
      BLI_assert(segments_in_chunk_num > 0);
      segments_per_chunk_cumulative.append(segments_per_chunk_cumulative.last() +
                                           segments_in_chunk_num);

      for (const int64_t segment_i :
           segments_in_chunks.index_range().take_back(segments_in_chunk_num)) {
        const std::variant<IndexRange, Span<T>> &segment = segments_in_chunks[segment_i];
        if (std::holds_alternative<IndexRange>(segment)) {
          /* No extra allocations necessary because static allocator is used. */
        }
        else {
          const Span<T> indices_in_segment = std::get<Span<T>>(segment);
          index_allocations_num += indices_in_segment.size();
        }
      }

      chunk.segments_num = segments_in_chunk_num;
    }

    if (chunks_to_postprocess.is_empty()) {
      return;
    }

    MutableSpan<int16_t> remaining_indices;
    MutableSpan<const int16_t *> remaining_indices_by_segment;
    MutableSpan<int16_t> remaining_cumulative_segment_sizes;

    {
      std::lock_guard lock{scope_mutex};
      remaining_indices_by_segment = allocator.allocate_array<const int16_t *>(
          segments_in_chunks.size());
      remaining_indices = allocator.allocate_array<int16_t>(index_allocations_num);
      remaining_cumulative_segment_sizes = allocator.allocate_array<int16_t>(
          segments_in_chunks.size() + chunks_to_postprocess.size());
    }

    const OffsetIndices<int64_t> segments_by_chunk = segments_per_chunk_cumulative.as_span();

    const auto take_front_and_drop = [](auto &span, const int64_t n) {
      auto front = span.take_front(n);
      BLI_assert(front.size() == n);
      span = span.drop_front(n);
      return front;
    };

    for (const int64_t i : chunks_to_postprocess.index_range()) {
      const int64_t index_in_slice = chunks_to_postprocess[i];
      const int64_t chunk_i = slice[index_in_slice];
      const IndexRange segments_in_chunk = segments_by_chunk[i];
      const int16_t segments_num = int16_t(segments_in_chunk.size());
      Chunk &chunk = chunks[chunk_i];
      const int64_t chunk_offset = chunk_ids[chunk_i] * chunk_capacity;

      MutableSpan<const int16_t *> indices_by_segment = take_front_and_drop(
          remaining_indices_by_segment, segments_num);
      MutableSpan<int16_t> cumulative_segment_sizes = take_front_and_drop(
          remaining_cumulative_segment_sizes, segments_num + 1);

      int64_t cumulative_size = 0;
      for (const int64_t segment_i : IndexRange(segments_num)) {
        const std::variant<IndexRange, Span<T>> &segment =
            segments_in_chunks[segments_in_chunk[segment_i]];
        cumulative_segment_sizes[segment_i] = int16_t(cumulative_size);

        if (std::holds_alternative<IndexRange>(segment)) {
          const IndexRange range_in_segment = std::get<IndexRange>(segment);
          indices_by_segment[segment_i] = static_offsets +
                                          (range_in_segment.first() - chunk_offset);
          cumulative_size += range_in_segment.size();
        }
        else {
          const Span<T> indices_in_segment = std::get<Span<T>>(segment);
          MutableSpan<int16_t> new_indices = take_front_and_drop(remaining_indices,
                                                                 indices_in_segment.size());
          for (const int64_t index_in_segment : new_indices.index_range()) {
            new_indices[index_in_segment] = int16_t(indices_in_segment[index_in_segment] -
                                                    chunk_offset);
          }
          indices_by_segment[segment_i] = new_indices.data();
          cumulative_size += indices_in_segment.size();
        }
      }
      cumulative_segment_sizes[segments_num] = int16_t(cumulative_size);

      chunk.indices_by_segment = indices_by_segment.data();
      chunk.cumulative_segment_sizes = cumulative_segment_sizes.data();
    }

    BLI_assert(remaining_indices.is_empty());
    BLI_assert(remaining_indices_by_segment.is_empty());
    BLI_assert(remaining_cumulative_segment_sizes.is_empty());
  });

  MutableSpan<int64_t> cumulative_chunk_sizes = allocator.allocate_array<int64_t>(chunks_num + 1);
  int64_t cumulative_size = 0;
  for (const int64_t i : chunks.index_range()) {
    cumulative_chunk_sizes[i] = cumulative_size;
    cumulative_size += chunks[i].size();
  }
  cumulative_chunk_sizes.last() = cumulative_size;

  IndexMask mask;
  IndexMaskData &mask_data = mask.data_for_inplace_construction();
  mask_data.chunks_num = chunks_num;
  mask_data.indices_num = indices.size();
  mask_data.chunks = chunks.data();
  mask_data.chunk_ids = chunk_ids.data();
  mask_data.cumulative_chunk_sizes = cumulative_chunk_sizes.data();
  mask_data.begin_it = {0, 0};
  mask_data.end_it = chunks.last().end_iterator();
  return mask;

  return {};
}

template<typename T> void from_index_mask(const IndexMask &mask, MutableSpan<T> r_indices)
{
  BLI_assert(mask.size() == r_indices.size());
  int64_t current_i = 0;
  mask.foreach_index([&](const int64_t index) mutable {
    r_indices[current_i] = T(index);
    current_i++;
  });
}

}  // namespace unique_sorted_indices

void IndexMask::foreach_span(FunctionRef<void(OffsetSpan<int64_t, int16_t>)> fn) const
{
  if (data_.indices_num == 0) {
    return;
  }

  int64_t chunk_i = 0;
  int64_t segment_i = data_.begin_it.segment_i;
  int64_t segment_drop_front = data_.begin_it.index_in_segment;
  const int64_t final_drop_back = data_.chunks[data_.chunks_num - 1].segment_size(
                                      data_.end_it.segment_i) -
                                  data_.end_it.index_in_segment;
  const int64_t final_segment_i = data_.end_it.segment_i;
  const int64_t final_segments_num = data_.end_it.segment_i + 1;

  while (chunk_i < data_.chunks_num) {
    const Chunk &chunk = data_.chunks[chunk_i];
    const int64_t chunk_id = data_.chunk_ids[chunk_i];
    const bool is_last_chunk = (chunk_i == data_.chunks_num - 1);
    const int64_t segments_num = is_last_chunk ? final_segments_num : chunk.segments_num;
    const int64_t offset = chunk_capacity * chunk_id;
    int64_t prev_cumulative_segment_size = chunk.cumulative_segment_sizes[segment_i];
    while (segment_i < segments_num) {
      const int64_t next_segment_i = segment_i + 1;
      const int64_t cumulative_segment_size = chunk.cumulative_segment_sizes[next_segment_i];
      const int64_t stored_segment_size = cumulative_segment_size - prev_cumulative_segment_size;
      const bool is_last_segment = is_last_chunk & (segment_i == final_segment_i);
      const int64_t segment_drop_back = is_last_segment * final_drop_back;
      const int16_t *indices_in_segment = chunk.indices_by_segment[segment_i] + segment_drop_front;
      const int64_t segment_size = stored_segment_size - segment_drop_front - segment_drop_back;
      const Span<int16_t> indices_span{indices_in_segment, segment_size};

      const OffsetSpan<int64_t, int16_t> segment{offset, indices_span};
      fn(segment);

      segment_drop_front = 0;
      segment_i = next_segment_i;
    }
    segment_i = 0;
    chunk_i++;
  }
}

static IndexMask bits_to_index_mask(const BitSpan bits,
                                    const int64_t start,
                                    LinearAllocator<> &allocator)
{
  Vector<int64_t> indices;
  for (const int64_t i : bits.index_range()) {
    if (bits[i]) {
      indices.append(i + start);
    }
  }
  return unique_sorted_indices::to_index_mask<int64_t>(indices, allocator);
}

static void index_mask_to_bits(const IndexMask &mask, const int64_t start, MutableBitSpan r_bits)
{
  BLI_assert(r_bits.size() >= mask.min_array_size() - start);
  r_bits.reset_all();
  mask.foreach_index([&](const int64_t i) { r_bits[i - start].set(); });
}

template<typename T>
IndexMask IndexMask::from_indices(const Span<T> indices, LinearAllocator<> &allocator)
{
  return unique_sorted_indices::to_index_mask(indices, allocator);
}

IndexMask IndexMask::from_bits(const BitSpan bits,
                               LinearAllocator<> &allocator,
                               const int64_t offset)
{
  return bits_to_index_mask(bits, offset, allocator);
}

static Set<int64_t> eval_expr(const Expr &base_expr, const IndexRange universe)
{
  Set<int64_t> result;
  switch (base_expr.type) {
    case Expr::Type::Atomic: {
      const AtomicExpr &expr = static_cast<const AtomicExpr &>(base_expr);
      expr.mask->foreach_index([&](const int64_t i) {
        BLI_assert(universe.contains(i));
        result.add_new(i);
      });
      break;
    }
    case Expr::Type::Union: {
      const UnionExpr &expr = static_cast<const UnionExpr &>(base_expr);
      for (const Expr *child : expr.children) {
        const Set<int64_t> child_result = eval_expr(*child, universe);
        for (const int64_t i : child_result) {
          result.add(i);
        }
      }
      break;
    }
    case Expr::Type::Difference: {
      const DifferenceExpr &expr = static_cast<const DifferenceExpr &>(base_expr);
      result = eval_expr(*expr.base, universe);
      for (const Expr *child : expr.children) {
        const Set<int64_t> child_result = eval_expr(*child, universe);
        for (const int64_t i : child_result) {
          result.remove(i);
        }
      }
      break;
    }
    case Expr::Type::Complement: {
      const ComplementExpr &expr = static_cast<const ComplementExpr &>(base_expr);
      const Set<int64_t> child_result = eval_expr(*expr.base, universe);
      for (const int64_t i : universe) {
        if (!child_result.contains(i)) {
          result.add_new(i);
        }
      }
      break;
    }
    case Expr::Type::Intersection: {
      const IntersectionExpr &expr = static_cast<const IntersectionExpr &>(base_expr);
      BLI_assert(!expr.children.is_empty());
      result = eval_expr(*expr.children.first(), universe);
      for (const Expr *child : expr.children.as_span().drop_front(1)) {
        const Set<int64_t> child_result = eval_expr(*child, universe);
        result.remove_if([&](const int64_t i) { return !child_result.contains(i); });
      }
      break;
    }
  }
  return result;
}

static Set<int64_t> find_chunk_ids_to_process(const Expr &base_expr, const IndexRange universe)
{
  Set<int64_t> result;
  switch (base_expr.type) {
    case Expr::Type::Atomic: {
      const AtomicExpr &expr = static_cast<const AtomicExpr &>(base_expr);
      for (const int64_t chunk_i : IndexRange(expr.mask->data().chunks_num)) {
        result.add_new(expr.mask->data().chunk_ids[chunk_i]);
      }
      break;
    }
    case Expr::Type::Union: {
      const UnionExpr &expr = static_cast<const UnionExpr &>(base_expr);
      for (const Expr *child : expr.children) {
        const Set<int64_t> child_result = find_chunk_ids_to_process(*child, universe);
        for (const int64_t chunk_id : child_result) {
          result.add(chunk_id);
        }
      }
      break;
    }
    case Expr::Type::Difference: {
      const DifferenceExpr &expr = static_cast<const DifferenceExpr &>(base_expr);
      result = find_chunk_ids_to_process(*expr.base, universe);
      break;
    }
    case Expr::Type::Complement: {
      const int64_t first_chunk_id = index_to_chunk_id(universe.first());
      const int64_t last_chunk_id = index_to_chunk_id(universe.last());
      for (const int64_t chunk_id :
           IndexRange(first_chunk_id, last_chunk_id - first_chunk_id + 1)) {
        result.add(chunk_id);
      }
      break;
    }
    case Expr::Type::Intersection: {
      const IntersectionExpr &expr = static_cast<const IntersectionExpr &>(base_expr);
      BLI_assert(!expr.children.is_empty());
      result = find_chunk_ids_to_process(*expr.children.first(), universe);
      for (const Expr *child : expr.children.as_span().drop_front(1)) {
        const Set<int64_t> child_result = find_chunk_ids_to_process(*child, universe);
        result.remove_if([&](const int64_t chunk_id) { return !child_result.contains(chunk_id); });
      }
      break;
    }
  }
  return result;
}

static void find_chunk_ids_to_process(const Expr &base_expr,
                                      const IndexRange universe,
                                      MutableBitSpan r_chunk_is_full,
                                      MutableBitSpan r_chunk_non_empty)
{
  using TmpBitVector = BitVector<1024>;

  const int64_t max_chunk_id = index_to_chunk_id(universe.last());
  const int64_t r_size = max_chunk_id + 1;
  BLI_assert(r_chunk_is_full.size() == r_size);
  BLI_assert(r_chunk_non_empty.size() == r_size);

  switch (base_expr.type) {
    case Expr::Type::Atomic: {
      const AtomicExpr &expr = static_cast<const AtomicExpr &>(base_expr);
      const IndexMaskData &data = expr.mask->data();
      if (data.chunks_num == 0) {
        break;
      }
      if (data.chunks_num == 1) {
        const int64_t chunk_id = data.chunk_ids[0];
        r_chunk_non_empty[chunk_id].set();
        if (data.indices_num == chunk_capacity) {
          r_chunk_is_full[data.chunk_ids[0]].set();
        }
        break;
      }
      for (const int64_t chunk_i : IndexRange(data.chunks_num)) {
        const int64_t chunk_id = data.chunk_ids[chunk_i];
        const Chunk &chunk = data.chunks[chunk_i];
        r_chunk_non_empty[chunk_id].set();
        MutableBitRef is_full = r_chunk_is_full[chunk_id];
        if (chunk.is_full()) {
          if (chunk_i == 0 && data.begin_it.segment_i == 0 &&
              data.begin_it.index_in_segment == 0) {
            is_full.set();
          }
          else if (chunk_i == data.chunks_num - 1 && data.end_it.segment_i == 0 &&
                   data.end_it.index_in_segment == chunk_capacity) {
            is_full.set();
          }
          else {
            is_full.set();
          }
        }
      }
      break;
    }
    case Expr::Type::Union: {
      const UnionExpr &expr = static_cast<const UnionExpr &>(base_expr);
      for (const Expr *child : expr.children) {
        TmpBitVector child_chunk_is_full(r_size, false);
        TmpBitVector child_chunk_non_empty(r_size, false);
        find_chunk_ids_to_process(*child, universe, child_chunk_is_full, child_chunk_non_empty);
        r_chunk_is_full |= child_chunk_is_full;
        r_chunk_non_empty |= child_chunk_non_empty;
      }
      break;
    }
    case Expr::Type::Difference: {
      const DifferenceExpr &expr = static_cast<const DifferenceExpr &>(base_expr);
      find_chunk_ids_to_process(*expr.base, universe, r_chunk_is_full, r_chunk_non_empty);
      for (const Expr *child : expr.children) {
        TmpBitVector child_chunk_is_full(r_size, false);
        TmpBitVector child_chunk_non_empty(r_size, false);
        find_chunk_ids_to_process(*child, universe, child_chunk_is_full, child_chunk_non_empty);
        r_chunk_is_full.clear_by_set_bits(child_chunk_non_empty);
        r_chunk_non_empty.clear_by_set_bits(child_chunk_is_full);
      }
      break;
    }
    case Expr::Type::Complement: {
      const ComplementExpr &expr = static_cast<const ComplementExpr &>(base_expr);
      /* The parameters are reversed intentionally. */
      find_chunk_ids_to_process(*expr.base, universe, r_chunk_non_empty, r_chunk_is_full);
      r_chunk_is_full.flip();
      r_chunk_non_empty.flip();
      break;
    }
    case Expr::Type::Intersection: {
      const IntersectionExpr &expr = static_cast<const IntersectionExpr &>(base_expr);
      BLI_assert(!expr.children.is_empty());
      find_chunk_ids_to_process(*expr.children[0], universe, r_chunk_is_full, r_chunk_non_empty);
      for (const Expr *child : expr.children.as_span().drop_front(1)) {
        TmpBitVector child_chunk_is_full(r_size, false);
        TmpBitVector child_chunk_non_empty(r_size, false);
        find_chunk_ids_to_process(*child, universe, child_chunk_is_full, child_chunk_non_empty);
        r_chunk_is_full &= child_chunk_is_full;
        r_chunk_non_empty &= child_chunk_non_empty;
      }
      break;
    }
  }
}

static Vector<int64_t> get_chunk_ids_to_evaluate_expression_in(const Expr & /*expr*/,
                                                               const IndexRange universe)
{
  const int64_t first_chunk_id = index_to_chunk_id(universe.first());
  const int64_t last_chunk_id = index_to_chunk_id(universe.last());
  Vector<int64_t> chunk_ids(last_chunk_id - first_chunk_id + 1);
  for (const int64_t i : chunk_ids.index_range()) {
    chunk_ids[i] = first_chunk_id + i;
  }
  return chunk_ids;
}

static std::optional<ChunkSlice> try_get_chunk_by_id(const IndexMask &mask, const int64_t chunk_id)
{
  const IndexMaskData &data = mask.data();
  const int64_t *chunk_id_iterator = std::lower_bound(
      data.chunk_ids, data.chunk_ids + data.chunks_num, chunk_id);
  const int64_t index = chunk_id_iterator - data.chunk_ids;
  if (index == data.chunks_num) {
    return std::nullopt;
  }
  if (data.chunk_ids[index] != chunk_id) {
    return std::nullopt;
  }
  ChunkSlice chunk_slice;
  chunk_slice.chunk = data.chunks + index;
  chunk_slice.begin_it = (index == 0) ? data.begin_it : RawChunkIterator{0, 0};
  chunk_slice.end_it = (index == data.chunks_num - 1) ? data.end_it :
                                                        chunk_slice.chunk->end_iterator();
  return chunk_slice;
}

static Set<int16_t> eval_expr_for_chunk_id__index_set(const Expr &base_expr,
                                                      const IndexRange universe,
                                                      const int64_t chunk_id)
{
  Set<int16_t> result;
  switch (base_expr.type) {
    case Expr::Type::Atomic: {
      const AtomicExpr &expr = static_cast<const AtomicExpr &>(base_expr);
      const IndexMask &mask = *expr.mask;
      const std::optional<ChunkSlice> chunk_slice = try_get_chunk_by_id(mask, chunk_id);
      if (!chunk_slice.has_value()) {
        break;
      }
      chunk_slice->foreach_span([&](const Span<int16_t> indices) {
        for (const int16_t index : indices) {
          result.add_new(index);
        }
      });
      break;
    }
    case Expr::Type::Union: {
      const UnionExpr &expr = static_cast<const UnionExpr &>(base_expr);
      for (const Expr *child : expr.children) {
        const Set<int16_t> child_result = eval_expr_for_chunk_id__index_set(
            *child, universe, chunk_id);
        for (const int16_t index : child_result) {
          result.add(index);
        }
      }
      break;
    }
    case Expr::Type::Difference: {
      const DifferenceExpr &expr = static_cast<const DifferenceExpr &>(base_expr);
      result = eval_expr_for_chunk_id__index_set(*expr.base, universe, chunk_id);
      for (const Expr *child : expr.children) {
        const Set<int16_t> child_result = eval_expr_for_chunk_id__index_set(
            *child, universe, chunk_id);
        result.remove_if([&](const int16_t index) { return child_result.contains(index); });
      }
      break;
    }
    case Expr::Type::Complement: {
      const ComplementExpr &expr = static_cast<const ComplementExpr &>(base_expr);
      const Set<int16_t> child_result = eval_expr_for_chunk_id__index_set(
          *expr.base, universe, chunk_id);
      for (const int64_t index : IndexRange(chunk_capacity)) {
        if (!child_result.contains(int16_t(index))) {
          result.add_new(int16_t(index));
        }
      }
      break;
    }
    case Expr::Type::Intersection: {
      const IntersectionExpr &expr = static_cast<const IntersectionExpr &>(base_expr);
      result = eval_expr_for_chunk_id__index_set(*expr.children[0], universe, chunk_id);
      for (const Expr *child : expr.children.as_span().drop_front(1)) {
        const Set<int16_t> child_result = eval_expr_for_chunk_id__index_set(
            *child, universe, chunk_id);
        result.remove_if([&](const int16_t index) { return !child_result.contains(index); });
      }
      break;
    }
  }
  return result;
}

static void eval_expressions_for_chunk_ids(const Expr &expr,
                                           const IndexRange universe,
                                           const Span<int64_t> chunk_ids,
                                           MutableSpan<Chunk> r_chunks,
                                           LinearAllocator<> &allocator,
                                           std::mutex &memory_mutex)
{
  BLI_assert(chunk_ids.size() == r_chunks.size());
  for (const int64_t chunk_i : chunk_ids.index_range()) {
    const int64_t chunk_id = chunk_ids[chunk_i];
    Chunk &chunk = r_chunks[chunk_i];

    const Set<int16_t> indices_in_chunk = eval_expr_for_chunk_id__index_set(
        expr, universe, chunk_id);

    MutableSpan<const int16_t *> indices_by_segment;
    MutableSpan<int16_t> indices_in_segment;
    MutableSpan<int16_t> cumulative_segment_sizes;

    {
      std::lock_guard lock{memory_mutex};
      indices_by_segment = allocator.allocate_array<const int16_t *>(1);
      indices_in_segment = allocator.allocate_array<int16_t>(indices_in_chunk.size());
      cumulative_segment_sizes = allocator.allocate_array<int16_t>(2);
    }
    indices_by_segment[0] = indices_in_segment.data();
    cumulative_segment_sizes[0] = 0;
    cumulative_segment_sizes[1] = int16_t(indices_in_chunk.size());
    int64_t counter = 0;
    for (const int16_t index : indices_in_chunk) {
      indices_in_segment[counter] = index;
      counter++;
    }
    std::sort(indices_in_segment.begin(), indices_in_segment.end());

    chunk.segments_num = 1;
    chunk.indices_by_segment = indices_by_segment.data();
    chunk.cumulative_segment_sizes = cumulative_segment_sizes.data();
  }
}

IndexMask IndexMask::from_expr(const Expr &expr,
                               const IndexRange universe,
                               LinearAllocator<> &allocator)
{
  if (universe.is_empty()) {
    return {};
  }

  const Vector<int64_t> possible_chunk_ids = get_chunk_ids_to_evaluate_expression_in(expr,
                                                                                     universe);
  Vector<Chunk> possible_chunks(possible_chunk_ids.size());
  std::mutex memory_mutex;
  threading::parallel_for(possible_chunk_ids.index_range(), 32, [&](const IndexRange range) {
    eval_expressions_for_chunk_ids(expr,
                                   universe,
                                   possible_chunk_ids.as_span().slice(range),
                                   possible_chunks.as_mutable_span().slice(range),
                                   allocator,
                                   memory_mutex);
  });

  Vector<int64_t> non_empty_chunks;
  for (const int64_t i : possible_chunks.index_range()) {
    const Chunk &chunk = possible_chunks[i];
    if (chunk.size() > 0) {
      non_empty_chunks.append(i);
    }
  }
  const int64_t chunks_num = non_empty_chunks.size();
  if (chunks_num == 0) {
    return {};
  }

  MutableSpan<Chunk> final_chunks = allocator.allocate_array<Chunk>(chunks_num);
  MutableSpan<int64_t> final_chunk_ids = allocator.allocate_array<int64_t>(chunks_num);
  MutableSpan<int64_t> final_cumulative_chunk_sizes = allocator.allocate_array<int64_t>(
      chunks_num + 1);
  int64_t counter = 0;
  for (const int64_t i : IndexRange(chunks_num)) {
    const int64_t i2 = non_empty_chunks[i];
    const Chunk &chunk = possible_chunks[i2];
    final_chunks[i] = chunk;
    final_chunk_ids[i] = possible_chunk_ids[i2];
    final_cumulative_chunk_sizes[i] = counter;
    counter += chunk.size();
  }
  final_cumulative_chunk_sizes.last() = counter;

  const Chunk &last_chunk = final_chunks.last();

  IndexMask mask;
  IndexMaskData &data = mask.data_for_inplace_construction();
  data.chunks_num = chunks_num;
  data.indices_num = counter;
  data.chunks = final_chunks.data();
  data.chunk_ids = final_chunk_ids.data();
  data.cumulative_chunk_sizes = final_cumulative_chunk_sizes.data();
  data.begin_it = RawChunkIterator{0, 0};
  data.end_it = last_chunk.end_iterator();

  return mask;
}

template<typename T> void IndexMask::to_indices(MutableSpan<T> r_indices) const
{
  unique_sorted_indices::from_index_mask(*this, r_indices);
}

void IndexMask::to_bits(MutableBitSpan r_bits, int64_t offset) const
{
  index_mask_to_bits(*this, offset, r_bits);
}

template IndexMask IndexMask::from_indices(Span<int32_t>, LinearAllocator<> &);
template IndexMask IndexMask::from_indices(Span<int64_t>, LinearAllocator<> &);
template void IndexMask::to_indices(MutableSpan<int32_t>) const;
template void IndexMask::to_indices(MutableSpan<int64_t>) const;

namespace unique_sorted_indices {
template Vector<IndexRange> split_by_chunk(const Span<int32_t> indices);
template Vector<IndexRange> split_by_chunk(const Span<int64_t> indices);
}  // namespace unique_sorted_indices

void do_benchmark(const int64_t total);
void do_benchmark(const int64_t /*total*/)
{
}

}  // namespace blender::index_mask
