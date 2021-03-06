#pragma once

#warning This header is deprecated; use drake/common/value.h instead.

// TODO(jwnimmer-tri) Remove this file (and its build rules) on 2019-06-01.

#include "drake/common/drake_deprecated.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {

using AbstractValue
    DRAKE_DEPRECATED(
        "Spell as drake::AbstractValue instead. "
        "This alias will be removed on 2019-06-01")
    = drake::AbstractValue;

template <typename T>
using Value
    DRAKE_DEPRECATED(
        "Spell as drake::AbstractValue instead. "
        "This alias will be removed on 2019-06-01")
    = drake::Value<T>;

}  // namespace systems

}  // namespace drake
