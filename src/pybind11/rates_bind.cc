#include "quest_gnc/diffeo/rates.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using quest_gnc::multirotor::Rates;

void rates_bind(py::module &m) {
  py::class_<Rates>(m, "Rates")
      .def(py::init<>())
      .def("GetPDotQDot", &Rates::GetPDotQDot)
      .def("GetPQ", &Rates::GetPQ)
      .def("ProjectYawDerivToBody", &Rates::ProjectYawDerivToBody);
}
