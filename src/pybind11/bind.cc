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

#include <pybind11/pybind11.h>
namespace py = pybind11;

void rates_bind(py::module &);
void body_frame_bind(py::module &);
void lee_control_bind(py::module &);
void imu_integ_bind(py::module &);

PYBIND11_MODULE(quest_gncpy, m) {
  rates_bind(m);
  body_frame_bind(m);
  lee_control_bind(m);
  imu_integ_bind(m);
}
