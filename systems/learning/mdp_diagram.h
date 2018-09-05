#pragma once
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/learning/reward_system_interface.h"
#include "drake/systems/learning/state_observer_interface.h"

namespace drake {
namespace systems {
namespace learning {

/**
 * A class that implements the (discrete time) Mdp interface using a system
 * whose underlying dynamics are continuous time.
 */
template <typename T>
class MdpDiagram : public Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MdpDiagram)

  MdpDiagram() {}

  /**
   * Builds the Diagram in two steps: connects the plant, observer, and rewards
   * that were added using AddPlant(), AddObserver(), and AddReward(), the calls
   * the DiagramBuilder's BuildInto() method. Must be called after AddPlant()
   * and AddReward().
   */
  void Build() {
    ConnectObserversAndRewards();
    builder_.BuildInto(this);
  }

  /**
   * Adds a generic System with a single input port and at least one output
   * port. The first output port must be the state. Can be called at most once.
   * @param plant unique pointer to the System. Ownership will be transferred.
   * @return Pointer to the added plant.
   */
  System<T>* AddPlant(std::unique_ptr<System<T>> plant) {
    DRAKE_DEMAND(plant->get_num_input_ports() == 1);
    DRAKE_DEMAND(plant->get_num_output_ports() >= 1);

    System<T>* plant_ptr =
        builder_.template AddSystem<System<T>>(std::move(plant));
    plant_ = plant_ptr;

    return plant_ptr;
  }

  /**
   * Adds a reward of type System<T>, which must be derived from
   * RewardSystemInterface. The reward system must have a single scalar output.
   * @param reward Unique pointer to the reward. Ownership will be transferred.
   * @return Pointer to the added reward.
   */
  System<T>* AddReward(std::unique_ptr<System<T>> reward) {
    DRAKE_DEMAND(dynamic_cast<RewardSystemInterface<T>*>(reward.get()));

    System<T>* reward_ptr =
        builder_.template AddSystem<System<T>>(std::move(reward));
    reward_systems_.push_back(
        dynamic_cast<RewardSystemInterface<T>*>(reward_ptr));

    return reward_ptr;
  }

  // TODO(pvarin): Add an AddReward() method that takes an std::function rather
  // than a RewardSystem

  /**
   * Adds an observer of type System<T>, which must be derived from
   * StateObserverInterface.
   * @param observer Unique pointer to the observer. Ownership will be
   * transferred.
   * @return Pointer to the added observer.
   */
  System<T>* AddObserver(std::unique_ptr<System<T>> observer) {
    DRAKE_DEMAND(dynamic_cast<StateObserverInterface<T>*>(observer.get()));

    System<T>* observer_ptr =
        builder_.template AddSystem<System<T>>(std::move(observer));
    observers_.push_back(
        dynamic_cast<StateObserverInterface<T>*>(observer_ptr));

    return observer_ptr;
  }

  const InputPort<T>& get_input_port_action() const {
    return this->get_input_port(input_port_action_);
  }

  const OutputPort<T>& get_output_port_reward() const {
    return this->get_output_port(output_port_reward_);
  }

  const OutputPort<T>& get_output_port_observation() const {
    return this->get_output_port(output_port_observation_);
  }

  /**
   * Returns the full state of the system (positions and velocities). This port
   * is designed to get the state for visualization and shouldn't be used for
   * learning.
   */
  const OutputPort<T>& get_output_port_state() const {
    return this->get_output_port(output_port_state_);
  }

 private:
  // The port numbers for the port accessors.
  int input_port_action_{-1};
  int output_port_reward_{-1};
  int output_port_observation_{-1};
  int output_port_state_{-1};

  // The underlying DiagramBuilder.
  systems::DiagramBuilder<T> builder_;

  // Pointer to the plant System.
  systems::System<T>* plant_{nullptr};

  // A vector of reward system pointers.
  std::vector<RewardSystemInterface<T>*> reward_systems_;

  // A vector of observer pointers.
  std::vector<StateObserverInterface<T>*> observers_;

  // Connects the underlying diagram before simulating
  void ConnectObserversAndRewards();
};

}  // namespace learning
}  // namespace systems
}  // namespace drake
