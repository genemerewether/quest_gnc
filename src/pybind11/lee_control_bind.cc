#include "quest_gnc/ctrl/lee_control.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using quest_gnc::multirotor::LeeControl;
using namespace pybind11::literals;

void lee_control_bind(py::module &m) {
  py::class_<LeeControl>(m, "LeeControl")
      .def(py::init<>())
      .def("SetOdometry", &LeeControl::SetOdometry,
           "x_w"_a, "w_q_b"_a, "v_b"_a, "omega_b"_a)
      .def("SetPositionDes", &LeeControl::SetPositionDes,
           "x_w__des"_a, "v_w__des"_a, "a_w__des"_a)
      .def("SetPositionAngAccelDes", &LeeControl::SetPositionAngAccelDes,
           "x_w__des"_a, "v_w__des"_a, "alpha_b__des"_a)
      .def("SetVelocityDes", &LeeControl::SetVelocityDes,
           "v_w__des"_a, "a_w__des"_a)
      .def("SetAttitudeDes", &LeeControl::SetAttitudeDes,
           "w_q_b__des"_a, "omega_b__des"_a)
      .def("SetAttitudeAngAccelDes", &LeeControl::SetAttitudeAngAccelDes,
           "w_q_b__des"_a, "omega_b__des"_a, "alpha_b__des"_a);
}
