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
