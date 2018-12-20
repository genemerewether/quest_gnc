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

#include "quest_gnc/sysid/signal_gen.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using quest_gnc::sysid::SignalGen;
using namespace pybind11::literals; // NOLINT()

void signal_gen_bind(py::module &m) { // NOLINT()
  py::class_<SignalGen>(m, "SignalGen")
      .def(py::init<>())

// ----------------------------------------------------------------------
// Output getters
// ----------------------------------------------------------------------

      .def("GetScalar", [](SignalGen &s) { double val = 0.0; double dvaldt = 0.0;
                                           int r = s.GetScalar(&val, &dvaldt);
                                           return std::make_tuple(val, dvaldt, r); })

      .def("GetVector", &SignalGen::GetVector, "val"_a)

      .def("GetSO3", &SignalGen::GetSO3, "q"_a, "omega"_a)

// ----------------------------------------------------------------------
// Input setters
// ----------------------------------------------------------------------

      .def("SetChirp", &SignalGen::SetChirp, "omega_i"_a, "omega_f"_a,
           "amplitude"_a, "nIter"_a, "dt"_a)

      .def("SetAxis", &SignalGen::SetUnitAxis, "axis"_a);
}
