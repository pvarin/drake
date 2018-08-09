#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/systems/framework/leaf_system.h"

namespace py = pybind11;

namespace drake {
namespace pydrake {

  PYBIND11_MODULE(kuka_iiwa_arm, m) {
    using namespace drake::examples::kuka_iiwa_arm;
    using namespace drake::systems;

    m.doc() = "Binding lcm system classes for the Kuka Iiwa arm";

    py::class_<IiwaCommandReceiver, LeafSystem<double>>(m, "IiwaCommandReceiver")
      .def(py::init<int>(), py::arg("num_joints")=kIiwaArmNumJoints)
      .def("set_initial_position", &IiwaCommandReceiver::set_initial_position,
           py::arg("context"), py::arg("x"));

    py::class_<IiwaCommandSender, LeafSystem<double>>(m, "IiwaCommandSender")
      .def(py::init<int>(), py::arg("num_joints")=kIiwaArmNumJoints);

    py::class_<IiwaStatusReceiver, LeafSystem<double>>(m, "IiwaStatusReceiver")
      .def(py::init<int>(), py::arg("num_joints")=kIiwaArmNumJoints);

    py::class_<IiwaStatusSender, LeafSystem<double>>(m, "IiwaStatusSender")
      .def(py::init<int>(), py::arg("num_joints")=kIiwaArmNumJoints);
  }

} // namespace pydrake
} // namespace drake