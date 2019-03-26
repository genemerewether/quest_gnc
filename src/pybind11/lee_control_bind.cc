// ======================================================================
// \author mereweth
//
// \copyright
// Copyright 2009-2018, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
//
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ======================================================================

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
      .def(py::init<>())

// ----------------------------------------------------------------------
// Parameter, model, and gain setters
// ----------------------------------------------------------------------

      .def("SetWorldParams", &LeeControl::SetWorldParams, "wParams"_a)
      .def("SetModel", &LeeControl::SetModel, "mrModel"_a)
      .def("SetGains", &LeeControl::SetGains,
           "k_x"_a, "k_v"_a, "k_R"_a, "k_omega"_a)

// ----------------------------------------------------------------------
// Thrust and moment getters
// ----------------------------------------------------------------------

      .def("GetAccelAngAccelCommand", &LeeControl::GetAccelAngAccelCommand,
           "a_w__comm"_a, "alpha_b__comm"_a)
      .def("GetAccelCommand", &LeeControl::GetAccelCommand,
           "a_w__comm"_a, "velOnly"_a)
      .def("GetAngAccelCommand", &LeeControl::GetAngAccelCommand,
           "alpha_b__comm"_a, "rpVelOnly"_a, "yawVelOnly"_a)

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
           "w_q_b__des"_a, "omega_b__des"_a,
	   "rpVelOnly"_a, "yawVelOnly"_a,
	   "doSaturation"_a)
      .def("SetAttitudeAngAccelDes", &LeeControl::SetAttitudeAngAccelDes,
           "w_q_b__des"_a, "omega_b__des"_a, "alpha_b__des"_a);
}
