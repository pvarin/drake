#include "drake/systems/learning/mdp_diagram.h"

#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {
namespace learning {

using drake::systems::Adder;
using drake::systems::Multiplexer;
using drake::systems::PassThrough;

template <typename T>
void MdpDiagram<T>::ConnectObserversAndRewards() {
  DRAKE_DEMAND(reward_systems_.size() > 0);

  // Expose the action port and connect to the plant.
  auto action = builder_.template AddSystem<PassThrough<T>>(
      plant_->get_input_port(0).size());
  input_port_action_ = builder_.ExportInput(action->get_input_port());
  builder_.Connect(action->get_output_port(), plant_->get_input_port(0));

  // Expose the state output port
  output_port_state_ = builder_.ExportOutput(plant_->get_output_port(0));

  // Connect the plant to the observers.
  for (auto observer : observers_) {
    builder_.Connect(plant_->get_output_port(0),
                     observer->get_input_port_state());
  }

  // Connect the state and the action to the observers.
  for (auto reward_system : reward_systems_) {
    builder_.Connect(plant_->get_output_port(0),
                     reward_system->get_input_port_state());
    builder_.Connect(action->get_output_port(),
                     reward_system->get_input_port_action());
  }

  // Expose the observation output.
  if (observers_.size() == 0) {
    // If there are no observers expose the state.
    output_port_observation_ =
        builder_.ExportOutput(plant_->get_output_port(0));
  } else if (observers_.size() == 1) {
    // If there is only one observer expose the observation.
    output_port_observation_ = builder_.ExportOutput(
        observers_.front()->get_output_port_observation());
  } else {
    // If there is more than one observer multiples the outputs.
    std::vector<int> observation_sizes;
    for (auto observer : observers_) {
      observation_sizes.push_back(
          observer->get_output_port_observation().size());
    }
    auto mux = builder_.template AddSystem<Multiplexer<T>>(observation_sizes);
    for (uint i = 0; i < observers_.size(); i++) {
      builder_.Connect(observers_[i]->get_output_port_observation(),
                       mux->get_input_port(i));
    }
    output_port_observation_ = builder_.ExportOutput(mux->get_output_port(0));
  }

  // Expose the reward output.
  if (reward_systems_.size() == 1) {
    output_port_reward_ = builder_.ExportOutput(
        reward_systems_.front()->get_output_port_reward());
  } else {
    // If there are more than one reward systems add them together.
    auto adder =
        builder_.template AddSystem<Adder<T>>(reward_systems_.size(), 1);
    for (uint i = 0; i < reward_systems_.size(); i++) {
      builder_.Connect(reward_systems_[i]->get_output_port_reward(),
                       adder->get_input_port(i));
    }
    output_port_reward_ = builder_.ExportOutput(adder->get_output_port());
  }
}

template class MdpDiagram<double>;

}  // namespace learning
}  // namespace systems
}  // namespace drake
