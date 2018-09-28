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

#include "quest_gnc/diffeo/body_frame.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using quest_gnc::multirotor::BodyFrame;
using namespace pybind11::literals; // NOLINT()

void body_frame_bind(py::module &m) { // NOLINT()
  py::class_<BodyFrame>(m, "BodyFrame")
      .def(py::init<>())
      .def("FromYawAccel", &BodyFrame::FromYawAccel,
           "yaw"_a, "accel_w"_a, "w_R_body"_a);
}
