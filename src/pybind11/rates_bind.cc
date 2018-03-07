#include "quest_gnc/diffeo/rates.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using quest_gnc::multirotor::Rates;
using namespace pybind11::literals;

void rates_bind(py::module &m) {
  py::class_<Rates>(m, "Rates")
      .def(py::init<>())
      .def("GetPDotQDot",
           [](const Rates &r,
              double thrust,
              const Eigen::Vector3d& jerk,
              const Eigen::Vector3d& snap,
              const Eigen::Matrix3d& w_R_body,
              const Eigen::Vector3d& omega) {
                   double pDot = 0.0f;
                   double qDot = 0.0f;
                   int stat = r.GetPDotQDot(thrust, jerk, snap, w_R_body, omega,
                                            pDot, qDot);
                   return std::make_tuple(pDot, qDot, stat);
           },
           "thrust"_a, "jerk"_a, "snap"_a, "w_R_body"_a, "omega"_a)
      .def("GetPQ",
           [](const Rates &r,
              double thrust,
              const Eigen::Vector3d& jerk,
              const Eigen::Matrix3d& w_R_body) {
                   double p = 0.0f;
                   double q = 0.0f;
                   int stat = r.GetPQ(thrust, jerk, w_R_body,
                                      p, q);
                   return std::make_tuple(p, q, stat);
           },
           "thrust"_a, "jerk"_a, "w_R_body"_a)
      .def("ProjectYawDerivToBody",
           [](const Rates &r,
              double yaw_deriv,
              const Eigen::Vector3d& zBody_w) {
                   double body_z_deriv = 0.0f;
                   int stat = r.ProjectYawDerivToBody(yaw_deriv, zBody_w,
                                                      yaw_deriv);
                   return std::make_tuple(yaw_deriv, stat);
           },
           "yaw_deriv"_a, "zBody_w"_a);
}

// m.def("foo", [](int i) { int rv = foo(i); return std::make_tuple(rv, i); });
