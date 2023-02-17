/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_span.hh"

namespace blender {

template<typename T, typename BaseT> class OffsetSpan {
 private:
  T offset_ = 0;
  Span<BaseT> data_;

 public:
  OffsetSpan(const T offset, const Span<BaseT> data) : offset_(offset), data_(data)
  {
  }

  Span<BaseT> base_span() const
  {
    return data_;
  }

  int64_t size() const
  {
    return data_.size();
  }

  T operator[](const int64_t i) const
  {
    return T(data_[i]) + offset_;
  }

  class Iterator {
   private:
    T offset_;
    const BaseT *data_;

   public:
    Iterator(const T offset, const BaseT *data) : offset_(offset), data_(data)
    {
    }

    Iterator &operator++()
    {
      data_++;
      return *this;
    }

    T operator*() const
    {
      return T(*data_) + offset_;
    }

    friend bool operator!=(const Iterator &a, const Iterator &b)
    {
      BLI_assert(a.offset_ == b.offset_);
      return a.data_ != b.data_;
    }
  };

  Iterator begin() const
  {
    return {offset_, data_.begin()};
  }

  Iterator end() const
  {
    return {offset_, data_.end()};
  }
};

}  // namespace blender
