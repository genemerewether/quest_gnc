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

#include "quest_gnc/est/imu_integ.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using quest_gnc::estimation::ImuInteg;
using quest_gnc::WorldParams;
using namespace pybind11::literals; // NOLINT()

void imu_integ_bind(py::module &m) { // NOLINT()
  py::class_<ImuInteg>(m, "ImuInteg")
      .def(py::init<>())

// ----------------------------------------------------------------------
// Parameter, model, and gain setters
// ----------------------------------------------------------------------

      .def("SetWorldParams", &ImuInteg::SetWorldParams, "wParams"_a);

// ----------------------------------------------------------------------
// Output getters
// ----------------------------------------------------------------------


// ----------------------------------------------------------------------
// Input setters
// ----------------------------------------------------------------------

}
