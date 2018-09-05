#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/learning/reward_system_interface.h"

namespace drake {
namespace systems {
namespace learning {

/**
 * Implements the RewardSystemInterface and computes a quadratic reward on the
 * actions and the state. The reward is computed as:
 *    reward = x'*Q*x + l_x'*x + u'*R*u + l_u'*u
 */
template <typename T>
class QuadraticReward final : public LeafSystem<T>,
                              public RewardSystemInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticReward)

  /**
   * Constructors with and without default cost terms.
   */
  QuadraticReward(Eigen::MatrixXd Q, Eigen::VectorXd l_x, Eigen::MatrixXd R,
                  Eigen::VectorXd l_u);

  /**
   * Returns the input port for the action.
   */
  const InputPort<T>& get_input_port_action() const override {
    return LeafSystem<T>::get_input_port(input_port_action_index_);
  }

  /**
   * Returns the input port for the state.
   */
  const InputPort<T>& get_input_port_state() const override {
    return LeafSystem<T>::get_input_port(input_port_state_index_);
  }

  /**
   * Returns the output port for the reward.
   */
  const OutputPort<T>& get_output_port_reward() const override {
    return LeafSystem<T>::get_output_port(output_port_reward_index_);
  }

 private:
  void CalcReward(const Context<T>& context, BasicVector<T>* reward) const;

  // State reward coefficients
  Eigen::MatrixXd Q_;
  Eigen::VectorXd l_x_;

  // Action reward coefficients
  Eigen::MatrixXd R_;
  Eigen::VectorXd l_u_;

  // port numbers
  int input_port_action_index_{-1};
  int input_port_state_index_{-1};
  int output_port_reward_index_{-1};
};

}  // namespace learning
}  // namespace systems
}  // namespace drake
