/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_geometry_nodes_bundle.hh"
#include "BKE_geometry_nodes_closure_fwd.hh"

#include "BLI_resource_scope.hh"

#include "FN_lazy_function.hh"

namespace blender::bke {

class ClosureSignature {
 private:
  std::shared_ptr<SocketListSignature> inputs_sockets_;
  std::shared_ptr<SocketListSignature> outputs_sockets_;

 public:
  ClosureSignature(std::shared_ptr<SocketListSignature> input_sockets,
                   std::shared_ptr<SocketListSignature> output_sockets)
      : inputs_sockets_(input_sockets), outputs_sockets_(output_sockets)
  {
  }

  const SocketListSignature &inputs_sockets() const
  {
    return *inputs_sockets_;
  }
  const SocketListSignature &outputs_sockets() const
  {
    return *outputs_sockets_;
  }
};

struct ClosureFunctionIndices {
  struct {
    IndexRange main;
    IndexRange output_usages;
    /** Output bsocket index -> input lf socket index. */
    Map<int, int> output_data_reference_sets;
  } inputs;
  struct {
    IndexRange main;
    IndexRange input_usages;
  } outputs;
};

class Closure : public ImplicitSharingInfo {
 private:
  std::shared_ptr<ClosureSignature> signature_;
  std::unique_ptr<ResourceScope> scope_;
  const fn::lazy_function::LazyFunction &function_;
  ClosureFunctionIndices indices_;

 public:
  Closure(std::shared_ptr<ClosureSignature> signature,
          std::unique_ptr<ResourceScope> scope,
          const fn::lazy_function::LazyFunction &function,
          ClosureFunctionIndices indices)
      : signature_(signature), scope_(std::move(scope)), function_(function), indices_(indices)
  {
  }

  const ClosureSignature &signature() const
  {
    return *signature_;
  }

  const ClosureFunctionIndices &indices() const
  {
    return indices_;
  }

  const fn::lazy_function::LazyFunction &function() const
  {
    return function_;
  }

  void delete_self_with_data() override
  {
    MEM_delete(this);
  }
};

}  // namespace blender::bke
