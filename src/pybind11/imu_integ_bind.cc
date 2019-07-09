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
#include "quest_gnc/utils/common.h"

#include "Eigen/Geometry"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using namespace quest_gnc;
using quest_gnc::estimation::ImuInteg;
using quest_gnc::WorldParams;
using namespace pybind11::literals; // NOLINT()

void imu_integ_bind(py::module &m) { // NOLINT()
  py::class_<ImuInteg>(m, "ImuInteg")
      .def(py::init<>())

// ----------------------------------------------------------------------
// Parameter, model, and gain setters
// ----------------------------------------------------------------------

      .def("SetWorldParams", &ImuInteg::SetWorldParams, "wParams"_a)

      .def("SetTimeStep", &ImuInteg::SetTimeStep, "dt"_a)
  
// ----------------------------------------------------------------------
// Output getters
// ----------------------------------------------------------------------
    
      .def("GetState", [](ImuInteg &i) { Vector3 x_w; Quaternion w_q_b;
                                         Vector3 v_b; Vector3 omega_b;
                                         Vector3 a_b;
                                         int r = i.GetState(&x_w, &w_q_b, &v_b, &omega_b, &a_b);
                                         const Matrix3 w_R_b(w_q_b);
                                         const Vector3 w_euler_b = w_R_b.eulerAngles(2, 1, 0);
					 Eigen::Vector4d w_quat_as_vec_b(w_q_b.x(),
					                                 w_q_b.y(),
					                                 w_q_b.z(),
					                                 w_q_b.w());
                                         return std::make_tuple(x_w,
								w_euler_b,
								w_quat_as_vec_b,
								v_b,
								omega_b,
                                a_b,
								r);
      })

// ----------------------------------------------------------------------
// Input setters
// ----------------------------------------------------------------------

      .def("AddImu", &ImuInteg::AddImu, "imu"_a)
      
      .def("SetUpdate", (int (ImuInteg::*)(double tValid,
					   const Vector3& x_w,
					   const Quaternion& w_q_b,
					   const Vector3& v_w,
					   const Vector3& wBias,
					   const Vector3& aBias)) &ImuInteg::SetUpdate,
	   "tValid"_a,
	   "x_w"_a, "w_q_b"_a, "v_w"_a,
	   "wBias"_a, "aBias"_a)
    
// ----------------------------------------------------------------------
// Processing functions
// ----------------------------------------------------------------------

      .def("PropagateState", &ImuInteg::PropagateState);
    
}
