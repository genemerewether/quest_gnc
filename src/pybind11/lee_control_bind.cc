#include "quest_gnc/ctrl/lee_control.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using quest_gnc::multirotor::LeeControl;
using quest_gnc::multirotor::MultirotorModel;
using quest_gnc::WorldParams;
using namespace pybind11::literals; // NOLINT()

void lee_control_bind(py::module &m) { // NOLINT()
  py::class_<LeeControl>(m, "LeeControl")
      .def(py::init<const Eigen::Vector3d&, const Eigen::Vector3d&,
                    const Eigen::Vector3d&, const Eigen::Vector3d&,
                    MultirotorModel, WorldParams>())

// ----------------------------------------------------------------------
// Thrust and moment getters
// ----------------------------------------------------------------------

      .def("GetAccelAngAccelCommand", &LeeControl::GetAccelAngAccelCommand,
           "a_w__comm"_a, "alpha_b__comm"_a)
      .def("GetAccelCommand", &LeeControl::GetAccelCommand,
           "a_w__comm"_a)
      .def("GetAngAccelCommand", &LeeControl::GetAngAccelCommand,
           "alpha_b__comm"_a)

// ----------------------------------------------------------------------
// Feedback setters
// ----------------------------------------------------------------------

      .def("SetOdometry", &LeeControl::SetOdometry,
           "x_w"_a, "w_q_b"_a, "v_b"_a, "omega_b"_a)
      .def("SetAttitudeAngVel", &LeeControl::SetAttitudeAngVel,
           "w_q_b"_a, "omega_b"_a)
      .def("SetPositionLinVel", &LeeControl::SetPositionLinVel,
           "x_w"_a, "v_b"_a)

// ----------------------------------------------------------------------
// Command setters
// ----------------------------------------------------------------------

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
