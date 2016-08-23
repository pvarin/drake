#pragma once

// This file is generated by a script.  Do not edit!
// See drake/examples/Cars/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/systems/framework/basic_state_and_output_vector.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

namespace drake {

/// Describes the row indices of a DrivingCommand.
struct DrivingCommandIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 3;

  // The index of each individual coordinate.
  static const int kSteeringAngle = 0;
  static const int kThrottle = 1;
  static const int kBrake = 2;
};

/// Specializes BasicStateAndOutputVector with specific getters and setters.
template <typename T>
class DrivingCommand : public systems::BasicStateAndOutputVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef DrivingCommandIndices K;

  /// Default constructor.  Sets all rows to zero.
  DrivingCommand() : systems::BasicStateAndOutputVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  const T steering_angle() const { return this->GetAtIndex(K::kSteeringAngle); }
  void set_steering_angle(const T& steering_angle) {
    this->SetAtIndex(K::kSteeringAngle, steering_angle);
  }
  const T throttle() const { return this->GetAtIndex(K::kThrottle); }
  void set_throttle(const T& throttle) {
    this->SetAtIndex(K::kThrottle, throttle);
  }
  const T brake() const { return this->GetAtIndex(K::kBrake); }
  void set_brake(const T& brake) { this->SetAtIndex(K::kBrake, brake); }
  //@}

  /// @name Implement the LCMVector concept
  //@{
  typedef drake::lcmt_driving_command_t LCMMessageType;
  static std::string channel() { return "DRIVING_COMMAND"; }
  //@}
};

template <typename ScalarType>
bool encode(const double& t, const DrivingCommand<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_driving_command_t& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.steering_angle = wrap.steering_angle();
  msg.throttle = wrap.throttle();
  msg.brake = wrap.brake();
  return true;
}

template <typename ScalarType>
bool decode(const drake::lcmt_driving_command_t& msg,
            // NOLINTNEXTLINE(runtime/references)
            double& t,
            // NOLINTNEXTLINE(runtime/references)
            DrivingCommand<ScalarType>& wrap) {
  t = static_cast<double>(msg.timestamp) / 1000.0;
  wrap.set_steering_angle(msg.steering_angle);
  wrap.set_throttle(msg.throttle);
  wrap.set_brake(msg.brake);
  return true;
}

}  // namespace drake
