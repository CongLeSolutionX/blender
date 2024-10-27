/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_base.hh"
#include "BLI_math_bits.h"
#include "BLI_task.hh"
#include "BLI_virtual_array.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_evaluate_in_space_cc {

template<typename T> std::ostream &operator<<(std::ostream &stream, Vector<T> data);

template<typename T, int num>
std::ostream &operator<<(std::ostream &stream, const std::array<T, num> &data)
{
  stream << "{";
  for (const int64_t i : IndexRange(num)) {
    stream << data[i] << (num - 1 == i ? "" : "\t");
  }
  stream << "}";
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, const Span<T> span)
{
  for (const int64_t i : span.index_range()) {
    stream << span[i] << (span.size() - 1 == i ? "" : "\t");
  }
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, MutableSpan<T> span)
{
  stream << span.as_span();
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, Vector<T> data)
{
  stream << data.as_span();
  return stream;
}

template<typename T> std::ostream &operator<<(std::ostream &stream, Array<T> data)
{
  stream << data.as_span();
  return stream;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Position").implicit_field(implicit_field_inputs::position);
  b.add_input<decl::Vector>("Value").supports_field().hide_value();

  b.add_input<decl::Int>("Power").default_value(2);

  b.add_input<decl::Int>("Level");

  b.add_input<decl::Float>("Precision")
      .subtype(PROP_FACTOR)
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f);

  b.add_output<decl::Vector>("Weighted Sum").field_source_reference_all();
  b.add_output<decl::Int>("Weighed Difference Sum").field_source_reference_all();

  b.add_output<decl::Int>("Index").field_source_reference_all();
}

struct AKDTree {
  Array<int> ranks;
  Array<int> rank_offsets;
  int brut_eforce_postfix;
  Array<int> indices;
}

static Array<int> akdt_ranks_for_total(const int total_elements)
{
  constexpr int min_rank = 8;
  Vector<int, 30> stack;
  
  int rank_iterator = total_elements >> min_rank;
  int rank_index = min_rank;
  while (rank_iterator > 0) {
    if (rank_iterator & 1) {
      stack.append(rank_index);
    }
    rank_index++;
    rank_iterator >>= 1;
  }
}

static AKDTree akdt_from_3d_positions(const Span<float3> positions)
{
  AKDTree tree;
  tree.ranks.reinitialize();
}

static int akdt_nesting_for_size(const int total_elements)
{
  int levels = 0;
  for (int total_iter = total_elements; total_iter > 2; total_iter = (total_iter + 1) / 2) {
    levels++;
  }
  return levels;
}

static int n_levels_sum(const int total_nesting)
{
  return int((int64_t(1) << (total_nesting + 1)) - 1);
}

static IndexRange level_range(const int nesting_i)
{
  const int start = n_levels_sum(nesting_i - 1);
  const int size = 1 << nesting_i;
  return IndexRange::from_begin_size(start, size);
}

static IndexRange level_data_range(const int nesting_i)
{
  const IndexRange range = level_range(nesting_i);
  return IndexRange::from_begin_size(range.start() + nesting_i * 2, range.size() + 2);
}

static void akdt_from_positions(const Span<float3> positions, MutableSpan<int> indices)
{
  array_utils::fill_index_range<int>(indices);

  Vector<MutableSpan<int>> indices_stack = {indices};
  Vector<int> axis_stack = {0};

  Vector<int> nesting_stack = {0};
  [[maybe_unused]] int max_nesting = 0;
  
  while (!indices_stack.is_empty()) {
    MutableSpan<int> indices = indices_stack.pop_last();
    const int axis_index = axis_stack.pop_last();

    [[maybe_unused]] const int nesting_level = nesting_stack.pop_last();
    max_nesting = std::max(max_nesting, nesting_level);

    std::sort(indices.begin(), indices.end(), [&](const int a, const int b) {
      return positions[a][axis_index] < positions[b][axis_index];
    });

    if (indices.size() < 3) {
      continue;
    }

    const int next_axis = math::mod_periodic(axis_index + 1, 3);

    indices_stack.append(indices.take_front(indices.size() / 2));
    axis_stack.append(next_axis);

    indices_stack.append(indices.drop_front(indices.size() / 2));
    axis_stack.append(next_axis);

    nesting_stack.append(nesting_level + 1);
    nesting_stack.append(nesting_level + 1);
  }
  
  // printf("%d, %d;\n", akdt_nesting_for_size(positions.size()), max_nesting);
  BLI_assert(akdt_nesting_for_size(positions.size()) == max_nesting);
}

static void akdt_segmentation_from_size(const int total_elements, Array<int> &r_offset_indices)
{
  BLI_assert(n_levels_sum(0) == (1));
  BLI_assert(n_levels_sum(1) == (1 + 2));
  BLI_assert(n_levels_sum(2) == (1 + 2 + 4));
  BLI_assert(n_levels_sum(3) == (1 + 2 + 4 + 8));
  BLI_assert(n_levels_sum(4) == (1 + 2 + 4 + 8 + 16));
  BLI_assert(n_levels_sum(5) == (1 + 2 + 4 + 8 + 16 + 32));
  BLI_assert(n_levels_sum(6) == (1 + 2 + 4 + 8 + 16 + 32 + 64));
  BLI_assert(n_levels_sum(7) == (1 + 2 + 4 + 8 + 16 + 32 + 64 + 128));

  BLI_assert(level_range(0) == IndexRange::from_begin_size(0, 1));
  BLI_assert(level_range(1) == IndexRange::from_begin_size(1, 2));
  BLI_assert(level_range(2) == IndexRange::from_begin_size(1 + 2, 4));
  BLI_assert(level_range(3) == IndexRange::from_begin_size(1 + 2 + 4, 8));
  BLI_assert(level_range(4) == IndexRange::from_begin_size(1 + 2 + 4 + 8, 16));
  BLI_assert(level_range(5) == IndexRange::from_begin_size(1 + 2 + 4 + 8 + 16, 32));
  
  
  
  const int total_nesting = akdt_nesting_for_size(total_elements);
  const int total_indices = level_data_range(total_nesting).one_after_last();
  r_offset_indices.reinitialize(total_indices);
  r_offset_indices.as_mutable_span().fill(-1);
  
  if (r_offset_indices.is_empty()) {
    return;
  }

  MutableSpan<int> level_indices = r_offset_indices.as_mutable_span().slice(level_data_range(0));
  r_offset_indices[0] = 0;
  r_offset_indices[1] = total_elements / 2;
  r_offset_indices[2] = total_elements;

  for (const int nesting_i : IndexRange(total_nesting).shift(1)) {
    printf(">> %d;\n", nesting_i);
    const auto aaa = level_data_range(nesting_i - 1);
    const Span<int> prev_level_indices = r_offset_indices.as_span().slice(aaa);
    const auto bbb = level_data_range(nesting_i);
    MutableSpan<int> level_indices = r_offset_indices.as_mutable_span().slice(bbb);
    std::cout << aaa << ";\n";
    std::cout << bbb << ";\n";
    
    level_indices.first() = 0;
    level_indices[1] = prev_level_indices[1] / 2;
    for (const int i : IndexRange(prev_level_indices.size() - 3)) {
      const int start = prev_level_indices[1 + i];
      const int end = prev_level_indices[2 + i];
      const int half = (end - start) / 2;

      level_indices[2 + i * 2 + 0] = start + half / 2;
      level_indices[2 + i * 2 + 1] = start + half + half / 2;
    }
    level_indices.last(1) = prev_level_indices.last(1) + (prev_level_indices.last(0) - prev_level_indices.last(1) + 1) / 2;
    level_indices.last() = total_elements;
  }
}

template<typename Func>
static void foreach_bottom_to_top(const int total_elements, const Func &func)
{
  Vector<IndexRange> stack;
  for (IndexRange range(total_elements); range.size() > 2; ) {
    const int half = range.size() / 2;
    stack.append(range.drop_front(half));
    stack.append(range.take_front(half));
    range = range.take_front(half);
  }

  

  while (!stack.is_empty()) {
    const IndexRange range = stack.pop_last();
    if (range.size() <= 4) {
      continue;
    }
    
  }
}

static void akdt_mean_sums(const Span<int> indices, const Span<int> start_indices, const Span<float3> src_values, MutableSpan<float3> dst_values)
{
  
}

class DifferenceSumFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_field_;
  const Field<float3> value_field_;

 public:
  DifferenceSumFieldInput(Field<float3> positions_field, Field<float3> value_field)
      : bke::GeometryFieldInput(CPPType::get<int>(), "Weighed Difference Sum"),
        positions_field_(std::move(positions_field)),
        value_field_(std::move(value_field))
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask &mask) const final
  {
    if (!context.attributes()) {
      return {};
    }
    const int domain_size = context.attributes()->domain_size(context.domain());
    fn::FieldEvaluator evaluator{context, domain_size};
    evaluator.add(positions_field_);
    evaluator.add(value_field_);
    evaluator.evaluate();
    const VArraySpan<float3> positions = evaluator.get_evaluated<float3>(0);
    const VArraySpan<float3> values = evaluator.get_evaluated<float3>(1);

    
    Array<int> indices(positions.size());
    akdt_from_positions(positions, indices);

    Array<int> start_indices;
    akdt_segmentation_from_size(positions.size(), start_indices);

    Array<float3> centres(n_levels_sum(akdt_nesting_for_size(positions.size())));
    akdt_mean_sums(indices, start_indices, positions, centres);
    
    

    return VArray<int>::ForContainer(std::move(indices));
  }

 public:
  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const
  {
    positions_field_.node().for_each_field_input_recursive(fn);
    value_field_.node().for_each_field_input_recursive(fn);
  }
};

class DifferenceSumIndexFieldInput final : public bke::GeometryFieldInput {
 private:
  const Field<float3> positions_field_;
  const Field<float3> value_field_;
  const int level_;

 public:
  DifferenceSumIndexFieldInput(Field<float3> positions_field, Field<float3> value_field, const int level)
      : bke::GeometryFieldInput(CPPType::get<int>(), "Weighed Difference Sum"),
        positions_field_(std::move(positions_field)),
        value_field_(std::move(value_field)),
        level_(level)
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask &mask) const final
  {
    if (!context.attributes()) {
      return {};
    }
    const int domain_size = context.attributes()->domain_size(context.domain());
    fn::FieldEvaluator evaluator{context, domain_size};
    evaluator.add(positions_field_);
    evaluator.add(value_field_);
    evaluator.evaluate();
    const VArraySpan<float3> positions = evaluator.get_evaluated<float3>(0);
    const VArraySpan<float3> values = evaluator.get_evaluated<float3>(1);

    
    Array<int> indices(positions.size());
    akdt_from_positions(positions, indices);

    Array<int> start_indices;
    akdt_segmentation_from_size(positions.size(), start_indices);

    Array<int> iter_indices(positions.size());
    
    std::cout << start_indices << ";\n";
    
    const OffsetIndices<int> fraction = start_indices.as_span().slice(level_range(level_));
    for (const int fraction_i : fraction.index_range()) {
      for (const int i : fraction[fraction_i].index_range()) {
        iter_indices[indices[fraction[fraction_i][i]]] = i;
      }
    }
    

    return VArray<int>::ForContainer(std::move(iter_indices));
  }

 public:
  void for_each_field_input_recursive(FunctionRef<void(const FieldInput &)> fn) const
  {
    positions_field_.node().for_each_field_input_recursive(fn);
    value_field_.node().for_each_field_input_recursive(fn);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<float3> position_field = params.extract_input<Field<float3>>("Position");
  Field<float3> value_field = params.extract_input<Field<float3>>("Value");

  if (params.output_is_required("Weighed Difference Sum")) {
    params.set_output(
        "Weighed Difference Sum",
        Field<int>(std::make_shared<DifferenceSumFieldInput>(position_field, value_field)));
  }

  if (params.output_is_required("Index")) {
    const int level = params.extract_input<int>("Level");
    params.set_output(
        "Index",
        Field<int>(std::make_shared<DifferenceSumIndexFieldInput>(position_field, value_field, level)));
  }

  if (params.output_is_required("Weighted Sum")) {
    params.set_default_remaining_outputs();
  }
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_EVALUATE_IN_SPACE, "Field in Space", NODE_CLASS_CONVERTER);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_evaluate_in_space_cc
