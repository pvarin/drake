#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/output_port.h"

namespace drake {
namespace systems {
namespace learning {

/**
 * Interface for a reward system for use with the POMDP framework. This class
 * needs to be extended by concrete implementations. It provides named accessors
 * to a action and state port input ports and the observation output port.
 */
template <typename T>
class RewardSystemInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RewardSystemInterface)

  /**
   * Returns the input port for the action.
   */
  virtual const InputPort<T>& get_input_port_action() const = 0;

  /**
   * Returns the input port for the state.
   */
  virtual const InputPort<T>& get_input_port_state() const = 0;

  /**
   * Returns the output port for the reward.
   */
  virtual const OutputPort<T>& get_output_port_reward() const = 0;

 protected:
  RewardSystemInterface() {
    DRAKE_DEMAND(this->get_output_port_reward().size() == 1);
  }
  virtual ~RewardSystemInterface() {}
};

}  // namespace learning
}  // namespace systems
}  // namespace drake
