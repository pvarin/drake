#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/learning/mdp_diagram.h"
#include "drake/systems/learning/quadratic_reward.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(learning, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::learning;

  py::class_<MdpDiagram<double>, systems::Diagram<double>>(m, "MdpDiagram")
      .def(py::init<>())
      .def("Build", &MdpDiagram<double>::Build)
      .def("AddPlant", &MdpDiagram<double>::AddPlant)
      .def("AddReward", &MdpDiagram<double>::AddReward)
      .def("AddObserver", &MdpDiagram<double>::AddObserver)
      .def("get_input_port_action", &MdpDiagram<double>::get_input_port_action,
           py::return_value_policy::reference_internal)
      .def("get_output_port_reward",
           &MdpDiagram<double>::get_output_port_reward,
           py::return_value_policy::reference_internal)
      .def("get_output_port_observation",
           &MdpDiagram<double>::get_output_port_observation,
           py::return_value_policy::reference_internal)
      .def("get_output_port_state", &MdpDiagram<double>::get_output_port_state,
           py::return_value_policy::reference_internal);

  py::class_<QuadraticReward<double>, systems::LeafSystem<double>>(
      m, "QuadraticReward")
      .def(py::init<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd,
                    Eigen::VectorXd>(),
           py::arg("Q"), py::arg("l_x"), py::arg("R"), py::arg("l_u"))
      .def("get_input_port_action",
           &QuadraticReward<double>::get_input_port_action,
           py::return_value_policy::reference_internal)
      .def("get_input_port_state",
           &QuadraticReward<double>::get_input_port_state,
           py::return_value_policy::reference_internal)
      .def("get_output_port_reward",
           &QuadraticReward<double>::get_output_port_reward,
           py::return_value_policy::reference_internal);
}

}  // namespace pydrake
}  // namespace drake
