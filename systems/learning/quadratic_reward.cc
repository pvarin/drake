#include "drake/systems/learning/quadratic_reward.h"

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace learning {

using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
QuadraticReward<T>::QuadraticReward(MatrixXd Q, VectorXd l_x, MatrixXd R,
                                    VectorXd l_u)
    : Q_(Q), l_x_(l_x), R_(R), l_u_(l_u) {
  DRAKE_DEMAND(Q.rows() == Q.cols());
  DRAKE_DEMAND(Q.rows() == l_x.size());
  DRAKE_DEMAND(R.rows() == R.cols());
  DRAKE_DEMAND(R.rows() == l_u.size());

  input_port_state_index_ =
      this->DeclareInputPort(kVectorValued, Q.rows()).get_index();
  input_port_action_index_ =
      this->DeclareInputPort(kVectorValued, R.rows()).get_index();
  output_port_reward_index_ =
      this->DeclareVectorOutputPort(BasicVector<T>(1),
                                    &QuadraticReward<T>::CalcReward)
          .get_index();
}

template <typename T>
void QuadraticReward<T>::CalcReward(const Context<T>& context,
                                    BasicVector<T>* reward) const {
  VectorX<T> state =
      this->EvalVectorInput(context, input_port_state_index_)->get_value();
  VectorX<T> action =
      this->EvalVectorInput(context, input_port_action_index_)->get_value();
  Vector1<T> r = state.transpose() * Q_ * state + l_x_.transpose() * state +
                 action.transpose() * R_ * action + l_u_.transpose() * action;
  reward->get_mutable_value() << r;
}

template class QuadraticReward<double>;

}  // namespace learning
}  // namespace systems
}  // namespace drake
