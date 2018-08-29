#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/output_port.h"

namespace drake {
namespace systems {
namespace learning {

/**
 * Interface for an observer system for use with the POMDP framework. This class
 * needs to be extended by concrete implementations. It provides named accessors
 * to a control input port and observation and state ports.
 */
template <typename T>
class StateObserverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateObserverInterface)

  /**
   * Returns the input port for the state.
   */
  virtual const InputPort<T>& get_input_port_state() const = 0;

  /**
   * Returns the output port for the observation.
   */
  virtual const OutputPort<T>& get_output_port_observation() const = 0;

 protected:
  StateObserverInterface() {}
  virtual ~StateObserverInterface() {}
};

}  // namespace learning
}  // namespace systems
}  // namespace drake
