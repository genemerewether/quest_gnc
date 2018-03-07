#include "quest_gnc/diffeo/body_frame.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using quest_gnc::multirotor::BodyFrame;
using namespace pybind11::literals;

void body_frame_bind(py::module &m) {
  py::class_<BodyFrame>(m, "BodyFrame")
      .def(py::init<>())
      .def("FromYawAccel", &BodyFrame::FromYawAccel,
           "yaw"_a, "accel"_a, "w_R_body"_a);
}
