/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_lazy_function.hh"

#include "BKE_compute_contexts.hh"
#include "BKE_geometry_nodes_closure.hh"
#include "BKE_geometry_nodes_reference_set.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_socket_value.hh"
#include "BKE_node_tree_reference_lifetimes.hh"

#include "DEG_depsgraph_query.hh"

namespace blender::nodes {

using bke::node_tree_reference_lifetimes::ReferenceSetInfo;
using bke::node_tree_reference_lifetimes::ReferenceSetType;

class LazyFunctionForClosureZone : public LazyFunction {
 private:
  const bNodeTree &btree_;
  const bke::bNodeTreeZone &zone_;
  const bNode &output_bnode_;
  const ZoneBuildInfo &zone_info_;
  const ZoneBodyFunction &body_fn_;
  std::shared_ptr<bke::ClosureSignature> closure_signature_;

 public:
  LazyFunctionForClosureZone(const bNodeTree &btree,
                             const bke::bNodeTreeZone &zone,
                             ZoneBuildInfo &zone_info,
                             const ZoneBodyFunction &body_fn)
      : btree_(btree),
        zone_(zone),
        output_bnode_(*zone.output_node),
        zone_info_(zone_info),
        body_fn_(body_fn)
  {
    debug_name_ = "Closure Zone";

    initialize_zone_wrapper(zone, zone_info, body_fn, false, inputs_, outputs_);
    for (const auto item : body_fn.indices.inputs.reference_sets.items()) {
      const ReferenceSetInfo &reference_set =
          btree.runtime->reference_lifetimes_info->reference_sets[item.key];
      if (reference_set.type == ReferenceSetType::ClosureInputReferenceSet) {
        BLI_assert(&reference_set.socket->owner_node() != zone_.input_node);
      }
      if (reference_set.type == ReferenceSetType::ClosureOutputData) {
        if (&reference_set.socket->owner_node() == zone_.output_node) {
          /* This reference set comes from the caller of the closure and is not captured at the
           * place where the closure is created. */
          continue;
        }
      }
      zone_info.indices.inputs.reference_sets.add_new(
          item.key,
          inputs_.append_and_get_index_as("Reference Set",
                                          CPPType::get<bke::GeometryNodesReferenceSet>()));
    }

    /* All border links are used. */
    for (const int i : zone_.border_links.index_range()) {
      inputs_[zone_info.indices.inputs.border_links[i]].usage = lf::ValueUsage::Used;
    }

    const auto &storage = *static_cast<const NodeGeometryClosureOutput *>(output_bnode_.storage);

    std::shared_ptr<bke::SocketListSignature> input_list =
        std::make_shared<bke::SocketListSignature>();
    std::shared_ptr<bke::SocketListSignature> output_list =
        std::make_shared<bke::SocketListSignature>();

    for (const int i : IndexRange(storage.input_items.items_num)) {
      const bNodeSocket &bsocket = zone_.input_node->output_socket(i);
      input_list->items.append({bsocket.typeinfo, bsocket.name});
    }
    for (const int i : IndexRange(storage.output_items.items_num)) {
      const bNodeSocket &bsocket = zone_.output_node->input_socket(i);
      output_list->items.append({bsocket.typeinfo, bsocket.name});
    }
    closure_signature_ = std::make_shared<bke::ClosureSignature>(std::move(input_list),
                                                                 std::move(output_list));
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    for (const int i : zone_.border_links.index_range()) {
      params.set_output(zone_info_.indices.outputs.border_link_usages[i], true);
    }

    const auto &storage = *static_cast<const NodeGeometryClosureOutput *>(output_bnode_.storage);

    std::unique_ptr<ResourceScope> closure_scope = std::make_unique<ResourceScope>();
    LinearAllocator<> &closure_allocator = closure_scope->linear_allocator();

    lf::Graph &lf_graph = closure_scope->construct<lf::Graph>("Closure Graph");
    lf::FunctionNode &lf_body_node = lf_graph.add_function(*body_fn_.function);
    bke::ClosureFunctionIndices closure_indices;

    for (const int i : IndexRange(storage.input_items.items_num)) {
      const NodeGeometryClosureInputItem &item = storage.input_items.items[i];
      const bNodeSocket &bsocket = zone_.input_node->output_socket(i);
      const CPPType &cpp_type = *bsocket.typeinfo->geometry_nodes_cpp_type;

      lf::GraphInputSocket &lf_graph_input = lf_graph.add_input(cpp_type, item.name);
      lf_graph.add_link(lf_graph_input, lf_body_node.input(body_fn_.indices.inputs.main[i]));

      lf::GraphOutputSocket &lf_graph_input_usage = lf_graph.add_output(
          CPPType::get<bool>(), "Usage: " + StringRef(item.name));
      lf_graph.add_link(lf_body_node.output(body_fn_.indices.outputs.input_usages[i]),
                        lf_graph_input_usage);
    }
    closure_indices.inputs.main = lf_graph.graph_inputs().index_range().take_back(
        storage.input_items.items_num);
    closure_indices.outputs.input_usages = lf_graph.graph_outputs().index_range().take_back(
        storage.input_items.items_num);

    for (const int i : IndexRange(storage.output_items.items_num)) {
      const NodeGeometryClosureOutputItem &item = storage.output_items.items[i];
      const bNodeSocket &bsocket = zone_.output_node->input_socket(i);
      const CPPType &cpp_type = *bsocket.typeinfo->geometry_nodes_cpp_type;

      lf::GraphOutputSocket &lf_graph_output = lf_graph.add_output(cpp_type, item.name);
      lf_graph.add_link(lf_body_node.output(body_fn_.indices.outputs.main[i]), lf_graph_output);

      lf::GraphInputSocket &lf_graph_output_usage = lf_graph.add_input(
          CPPType::get<bool>(), "Usage: " + StringRef(item.name));
      lf_graph.add_link(lf_graph_output_usage,
                        lf_body_node.input(body_fn_.indices.inputs.output_usages[i]));
    }
    closure_indices.outputs.main = lf_graph.graph_outputs().index_range().take_back(
        storage.output_items.items_num);
    closure_indices.inputs.output_usages = lf_graph.graph_inputs().index_range().take_back(
        storage.output_items.items_num);

    for (const int i : zone_.border_links.index_range()) {
      const CPPType &cpp_type = *zone_.border_links[i]->tosock->typeinfo->geometry_nodes_cpp_type;
      void *input_ptr = params.try_get_input_data_ptr(zone_info_.indices.inputs.border_links[i]);
      void *stored_ptr = closure_allocator.allocate(cpp_type.size(), cpp_type.alignment());
      cpp_type.move_construct(input_ptr, stored_ptr);
      if (!cpp_type.is_trivially_destructible()) {
        closure_scope->add_destruct_call(
            [&cpp_type, stored_ptr]() { cpp_type.destruct(stored_ptr); });
      }
      lf_body_node.input(body_fn_.indices.inputs.border_links[i]).set_default_value(stored_ptr);
    }

    for (const auto &item : body_fn_.indices.inputs.reference_sets.items()) {
      const ReferenceSetInfo &reference_set =
          btree_.runtime->reference_lifetimes_info->reference_sets[item.key];
      if (reference_set.type == ReferenceSetType::ClosureOutputData) {
        const bNodeSocket &socket = *reference_set.socket;
        const bNode &node = socket.owner_node();
        if (&node == zone_.output_node) {
          /* This reference set is passed in by the code that invokes the closure. */
          lf::GraphInputSocket &lf_graph_input = lf_graph.add_input(
              CPPType::get<bke::GeometryNodesReferenceSet>(),
              StringRef("Reference Set: ") + reference_set.socket->name);
          lf_graph.add_link(
              lf_graph_input,
              lf_body_node.input(body_fn_.indices.inputs.reference_sets.lookup(item.key)));
          closure_indices.inputs.output_data_reference_sets.add_new(reference_set.socket->index(),
                                                                    lf_graph_input.index());
          continue;
        }
      }

      auto &input_reference_set = *params.try_get_input_data_ptr<bke::GeometryNodesReferenceSet>(
          zone_info_.indices.inputs.reference_sets.lookup(item.key));
      auto &stored = closure_scope->construct<bke::GeometryNodesReferenceSet>(
          std::move(input_reference_set));
      lf_body_node.input(body_fn_.indices.inputs.reference_sets.lookup(item.key))
          .set_default_value(&stored);
    }

    bNodeTree &btree_orig = *reinterpret_cast<bNodeTree *>(
        DEG_get_original_id(const_cast<ID *>(&btree_.id)));
    if (btree_orig.runtime->logged_zone_graphs) {
      std::lock_guard lock{btree_orig.runtime->logged_zone_graphs->mutex};
      btree_orig.runtime->logged_zone_graphs->graph_by_zone_id.lookup_or_add_cb(
          output_bnode_.identifier, [&]() { return lf_graph.to_dot(); });
    }

    lf_graph.update_node_indices();

    lf::GraphExecutor &lf_graph_executor = closure_scope->construct<lf::GraphExecutor>(
        lf_graph, nullptr, nullptr, nullptr);

    bke::ClosurePtr closure{MEM_new<bke::Closure>(__func__,
                                                  closure_signature_,
                                                  std::move(closure_scope),
                                                  lf_graph_executor,
                                                  closure_indices)};

    params.set_output(zone_info_.indices.outputs.main[0],
                      bke::SocketValueVariant(std::move(closure)));
  }
};

struct EvaluateClosureEvalStorage {
  ResourceScope scope;
  bke::ClosurePtr closure;
  lf::Graph graph;
  std::optional<lf::GraphExecutor> graph_executor;
  void *graph_executor_storage = nullptr;
};

class LazyFunctionForEvaluateClosureNode : public LazyFunction {
 private:
  const bNodeTree &btree_;
  const bNode &bnode_;

 public:
  EvaluateClosureFunctionIndices indices_;

 public:
  LazyFunctionForEvaluateClosureNode(const bNode &bnode)
      : btree_(bnode.owner_tree()), bnode_(bnode)
  {
    debug_name_ = bnode.name;
    for (const int i : bnode.input_sockets().index_range().drop_back(1)) {
      const bNodeSocket &bsocket = bnode.input_socket(i);
      indices_.inputs.main.append(inputs_.append_and_get_index_as(
          bsocket.name, *bsocket.typeinfo->geometry_nodes_cpp_type, lf::ValueUsage::Maybe));
      indices_.outputs.input_usages.append(
          outputs_.append_and_get_index_as("Usage", CPPType::get<bool>()));
    }
    /* The closure input is always used. */
    inputs_[indices_.inputs.main[0]].usage = lf::ValueUsage::Used;
    for (const int i : bnode.output_sockets().index_range().drop_back(1)) {
      const bNodeSocket &bsocket = bnode.output_socket(i);
      indices_.outputs.main.append(outputs_.append_and_get_index_as(
          bsocket.name, *bsocket.typeinfo->geometry_nodes_cpp_type));
      indices_.inputs.output_usages.append(
          inputs_.append_and_get_index_as("Usage", CPPType::get<bool>()));
    }
    /* TODO: Reference sets. */
  }

  void *init_storage(LinearAllocator<> &allocator) const override
  {
    return allocator.construct<EvaluateClosureEvalStorage>().release();
  }

  void destruct_storage(void *storage) const override
  {
    auto *s = static_cast<EvaluateClosureEvalStorage *>(storage);
    if (s->graph_executor_storage) {
      s->graph_executor->destruct_storage(s->graph_executor_storage);
    }
    std::destroy_at(s);
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    const ScopedNodeTimer node_timer{context, bnode_};

    auto &eval_storage = *static_cast<EvaluateClosureEvalStorage *>(context.storage);

    if (!eval_storage.graph_executor) {
      eval_storage.closure = params.extract_input<bke::SocketValueVariant>(indices_.inputs.main[0])
                                 .extract<bke::ClosurePtr>();
      if (!eval_storage.closure) {
        for (const bNodeSocket *bsocket : bnode_.output_sockets().drop_back(1)) {
          const int index = bsocket->index();
          set_default_value_for_output_socket(params, indices_.outputs.main[index], *bsocket);
          params.set_output(indices_.outputs.input_usages[index], false);
        }
        return;
      }
      this->initialize_execution_graph(eval_storage);
    }

    lf::Context eval_graph_context{
        eval_storage.graph_executor_storage, context.user_data, context.local_user_data};
    eval_storage.graph_executor->execute(params, eval_graph_context);
  }

  void initialize_execution_graph(EvaluateClosureEvalStorage &eval_storage) const
  {
    const auto &node_storage = *static_cast<const NodeGeometryEvaluateClosure *>(bnode_.storage);

    lf::Graph &lf_graph = eval_storage.graph;

    for (const lf::Input &input : inputs_) {
      lf_graph.add_input(*input.type, input.debug_name);
    }
    for (const lf::Output &output : outputs_) {
      lf_graph.add_output(*output.type, output.debug_name);
    }
    const Span<lf::GraphInputSocket *> lf_graph_inputs = lf_graph.graph_inputs();
    const Span<lf::GraphOutputSocket *> lf_graph_outputs = lf_graph.graph_outputs();

    const bke::Closure &closure = *eval_storage.closure;
    const bke::ClosureSignature &closure_signature = closure.signature();
    const bke::ClosureFunctionIndices &closure_indices = closure.indices();

    bke::SocketListSignature eval_inputs;
    bke::SocketListSignature eval_outputs;
    for (const int i : IndexRange(node_storage.input_items.items_num)) {
      const auto &item = node_storage.input_items.items[i];
      const char *idname = bke::node_static_socket_type(item.socket_type, 0);
      const bke::bNodeSocketType *stype = bke::node_socket_type_find(idname);
      eval_inputs.items.append({stype, item.name});
    }
    for (const int i : IndexRange(node_storage.output_items.items_num)) {
      const auto &item = node_storage.output_items.items[i];
      const char *idname = bke::node_static_socket_type(item.socket_type, 0);
      const bke::bNodeSocketType *stype = bke::node_socket_type_find(idname);
      eval_outputs.items.append({stype, item.name});
    }

    Array<std::optional<int>> inputs_map(node_storage.input_items.items_num);
    bke::get_socket_list_signature_map(
        closure_signature.inputs_sockets(), eval_inputs, inputs_map);

    Array<std::optional<int>> outputs_map(node_storage.output_items.items_num);
    bke::get_socket_list_signature_map(
        closure_signature.outputs_sockets(), eval_outputs, outputs_map);

    lf::FunctionNode &lf_closure_node = lf_graph.add_function(closure.function());

    static constexpr bool static_true = true;
    static constexpr bool static_false = false;
    /* The closure input is always used. */
    lf_graph_outputs[indices_.outputs.input_usages[0]]->set_default_value(&static_true);

    for (const int input_item_i : IndexRange(node_storage.input_items.items_num)) {
      if (const std::optional<int> mapped_i = inputs_map[input_item_i]) {
        lf_graph.add_link(*lf_graph_inputs[indices_.inputs.main[input_item_i + 1]],
                          lf_closure_node.input(closure_indices.inputs.main[*mapped_i]));
        lf_graph.add_link(lf_closure_node.output(closure_indices.outputs.input_usages[*mapped_i]),
                          *lf_graph_outputs[indices_.outputs.input_usages[input_item_i + 1]]);
      }
      else {
        lf_graph_outputs[indices_.outputs.input_usages[input_item_i + 1]]->set_default_value(
            &static_false);
      }
    }

    for (const int output_item_i : IndexRange(node_storage.output_items.items_num)) {
      if (const std::optional<int> mapped_i = outputs_map[output_item_i]) {
        lf_graph.add_link(lf_closure_node.output(closure_indices.outputs.main[*mapped_i]),
                          *lf_graph_outputs[indices_.outputs.main[output_item_i]]);
        lf_graph.add_link(*lf_graph_inputs[indices_.inputs.output_usages[output_item_i]],
                          lf_closure_node.input(closure_indices.inputs.output_usages[*mapped_i]));
      }
      else {
        const bke::bNodeSocketType &stype = *eval_outputs.items[output_item_i].socket_type;
        const CPPType &type = *stype.geometry_nodes_cpp_type;
        void *fallback_value = eval_storage.scope.linear_allocator().allocate(type.size(),
                                                                              type.alignment());
        construct_socket_default_value(stype, fallback_value);
        lf_graph_outputs[indices_.outputs.main[output_item_i]]->set_default_value(fallback_value);
        if (!type.is_trivially_destructible()) {
          eval_storage.scope.add_destruct_call(
              [fallback_value, &type]() { type.destruct(fallback_value); });
        }
      }
    }

    for (const int i : closure_indices.inputs.main.index_range()) {
      lf::InputSocket &lf_closure_input = lf_closure_node.input(closure_indices.inputs.main[i]);
      if (lf_closure_input.origin()) {
        /* Handled already. */
        continue;
      }
      const bke::bNodeSocketType &stype = *closure_signature.inputs_sockets().items[i].socket_type;
      const CPPType &type = *stype.geometry_nodes_cpp_type;
      void *fallback_value = eval_storage.scope.linear_allocator().allocate(type.size(),
                                                                            type.alignment());
      construct_socket_default_value(stype, fallback_value);
      lf_closure_input.set_default_value(fallback_value);
      if (!type.is_trivially_destructible()) {
        eval_storage.scope.add_destruct_call(
            [fallback_value, &type]() { type.destruct(fallback_value); });
      }
    }

    for (const int i : closure_indices.outputs.main.index_range()) {
      lf::OutputSocket &lf_closure_output = lf_closure_node.output(
          closure_indices.outputs.main[i]);
      if (!lf_closure_output.targets().is_empty()) {
        /* Handled already. */
        continue;
      }
      lf_closure_node.input(closure_indices.inputs.output_usages[i])
          .set_default_value(&static_false);
    }

    for (const auto item : closure_indices.inputs.output_data_reference_sets.items()) {
      /* TODO */
      static const bke::GeometryNodesReferenceSet static_empty_reference_set;
      lf_closure_node.input(item.value).set_default_value(&static_empty_reference_set);
    }

    lf_graph.update_node_indices();
    eval_storage.graph_executor.emplace(lf_graph, nullptr, nullptr, nullptr);
    eval_storage.graph_executor_storage = eval_storage.graph_executor->init_storage(
        eval_storage.scope.linear_allocator());

    /* Log graph for debugging purposes. */
    bNodeTree &btree_orig = *reinterpret_cast<bNodeTree *>(
        DEG_get_original_id(const_cast<ID *>(&btree_.id)));
    if (btree_orig.runtime->logged_zone_graphs) {
      std::lock_guard lock{btree_orig.runtime->logged_zone_graphs->mutex};
      btree_orig.runtime->logged_zone_graphs->graph_by_zone_id.lookup_or_add_cb(
          bnode_.identifier, [&]() { return lf_graph.to_dot(); });
    }
  }
};

LazyFunction &build_closure_zone_lazy_function(ResourceScope &scope,
                                               const bNodeTree &btree,
                                               const bke::bNodeTreeZone &zone,
                                               ZoneBuildInfo &zone_info,
                                               const ZoneBodyFunction &body_fn)
{
  return scope.construct<LazyFunctionForClosureZone>(btree, zone, zone_info, body_fn);
}

EvaluateClosureFunction build_evaluate_closure_node_lazy_function(ResourceScope &scope,
                                                                  const bNode &bnode)
{
  EvaluateClosureFunction info;
  auto &fn = scope.construct<LazyFunctionForEvaluateClosureNode>(bnode);
  info.lazy_function = &fn;
  info.indices = fn.indices_;
  return info;
}

}  // namespace blender::nodes
