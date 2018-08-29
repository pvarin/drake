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
 * A wrapper class around DiagramBuilder that facilitates diagram building for
 * a POMDP diagram. This class provides three utilities: adding /
 * accessing a plant system, RewardSystemInterface and StateObserverInterface.
 * Access to a mutable DiagramBuilder is provided by get_mutable_builder().
 */
template <typename T>
class PomdpDiagramBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PomdpDiagramBuilder)

  PomdpDiagramBuilder() {}

  /**
   * Builds a Diagram in two steps: connects all the
   * StateFeedbackControllerInterface
   * added using AddController() with the RigidBodyPlant added using
   * AddPlant(), and connects the DrakeVisualizer if available with the plant.
   * Then calls DiagramBuilder's Build() method and returns the resulting
   * diagram. Must be called after AddPlant().
   */
  std::unique_ptr<systems::Diagram<T>> Build() {
    ConnectObserversAndRewards();
    return builder_.Build();
  }

  /**
   * Builds a Diagram in two steps: connects all the
   * StateFeedbackControllerInterface
   * added using AddController() with the RigidBodyPlant added using
   * AddPlant(), and connects the DrakeVisualizer if available with the plant.
   * Then calls DiagramBuilder's BuildInto() method. Must be called after
   * AddPlant().
   * @param[out] target Pointer to the resulting diagram.
   */
  void BuildInto(systems::Diagram<T>* target) {
    ConnectObserversAndRewards();
    builder_.BuildInto(target);
  }

  /**
   * Adds a generic System with a single input port and at least one output
   * port. The first output port must be the state. Can be called at most once.
   * @param plant unique pointer to the System. Ownership will be transferred.
   * @return Pointer to the added plant.
   */
  template <class PlantType>
  PlantType* AddPlant(std::unique_ptr<PlantType> plant) {
    DRAKE_DEMAND(dynamic_cast<systems::System<T>>(plant.get()));
    DRAKE_DEMAND(plant.get_num_input_ports() == 1);
    DRAKE_DEMAND(plant.get_num_output_ports() >= 1);

    PlantType* plant_ptr =
        builder_.template AddSystem<PlantType>(std::move(plant));
    plant_ = plant_ptr;

    return plant_ptr;
  }

  /**
   * Adds a reward of type RewardType, which must be derived from
   * RewardSystemInterface. The reward system must have a single scalar output.
   * @param reward Unique pointer to the reward. Ownership will be transferred.
   * @return Pointer to the added reward.
   */
  template <class RewardType>
  RewardType* AddReward(std::unique_ptr<RewardType> reward) {
    DRAKE_DEMAND(dynamic_cast<RewardSystemInterface<T>*>(reward.get()));

    RewardType* reward_ptr =
        builder_.template AddSystem<RewardType>(std::move(reward));
    reward_systems_.push_back(reward_ptr);

    return reward_ptr;
  }

  /**
   * Adds an observer of type ObserverType, which must be derived from
   * StateObserverInterface.
   * @param observer Unique pointer to the observer. Ownership will be
   * transferred.
   * @return Pointer to the added observer.
   */
  template <class ObserverType>
  ObserverType* AddObserver(std::unique_ptr<ObserverType> observer) {
    DRAKE_DEMAND(dynamic_cast<StateObserverInterface<T>*>(observer.get()));

    ObserverType* observer_ptr =
        builder_.template AddSystem<ObserverType>(std::move(observer));
    observers_.push_back(observer_ptr);

    return observer_ptr;
  }

  /**
   * Returns a vector of RewardSystemInterface pointers for the current rewards
   * instance @p instance_id.
   */
  std::vector<RewardSystemInterface<T>*> get_rewards() const {
    return reward_systems_;
  }

  /**
   * Returns a vector of RewardSystemInterface pointers for the current rewards
   * instance @p instance_id.
   */
  std::vector<StateObserverInterface<T>*> get_observers() const {
    return observers_;
  }

  /**
   * Returns a pointer to the plant System.
   */
  systems::System<T>* get_plant() const { return plant_; }

  /**
   * Returns a pointer to the underlying DiagramBuilder.
   */
  systems::DiagramBuilder<T>* get_mutable_builder() { return &builder_; }

 private:
  // Connects the plant output to the reward systems and the observers. Sums the
  // rewards and multiplexes the observers. If there are no observers it
  // connects the state output to the observation port.
  void ConnectObserversAndRewards();

  // The underlying DiagramBuilder.
  systems::DiagramBuilder<T> builder_;

  // Pointer to the added System.
  systems::System<T>* plant_{nullptr};

  // A vector of reward system pointers.
  std::vector<RewardSystemInterface<T>*> reward_systems_;

  // A vector of observer pointers.
  std::vector<StateObserverInterface<T>*> observers_;
};

}  // namespace learning
}  // namespace systems
}  // namespace drake
