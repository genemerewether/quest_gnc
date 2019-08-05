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
#include "quest_gnc/utils/common.h"

#include <Eigen/Geometry>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using namespace quest_gnc;
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

      // .def("GetAccelAngAccelCommand", &LeeControl::GetAccelAngAccelCommand,
      //      "a_w__comm"_a, "alpha_b__comm"_a)
      .def("GetAccelCommand", &LeeControl::GetAccelCommand,
           "a_w__comm"_a, "velOnly"_a)
      // .def("GetAngAccelCommand", &LeeControl::GetAngAccelCommand,
      //      "alpha_b__comm"_a, "rpVelOnly"_a, "yawVelOnly"_a)
      // added
      // .def("GetState", &LeeControl::GetState, 
      //       "x_w"_a, "w_R_b"_a, "v_b"_a, "omega_b"_a, "a_b"_a)
      // added
      // .def("GetDesired", &LeeControl::GetDesired,
      //       "x_w__des"_a, "w_q_b__des"_a, "v_b__des"_a, "omega_b__des"_a)

// ----------------------------------------------------------------------
// Feedback setters
// ----------------------------------------------------------------------

      // .def("SetOdometry", &LeeControl::SetOdometry,
      //      "x_w"_a, "w_q_b"_a, "v_b"_a, "omega_b"_a)
      // .def("SetAttitudeAngVel", &LeeControl::SetAttitudeAngVel,
      //      "w_q_b"_a, "omega_b"_a)
      .def("SetPositionLinVelAcc", &LeeControl::SetPositionLinVelAcc,
           "x_w"_a, "v_b"_a, "a_b"_a)

// ----------------------------------------------------------------------
// Command setters
// ----------------------------------------------------------------------

      .def("SetPositionDes", &LeeControl::SetPositionDes,
           "x_w__des"_a, "v_w__des"_a, "a_w__des"_a, "j_w__des"_a)
      .def("SetPositionAngAccelDes", &LeeControl::SetPositionAngAccelDes,
           "x_w__des"_a, "v_w__des"_a, "alpha_b__des"_a)
      .def("SetVelocityDes", &LeeControl::SetVelocityDes,
           "v_w__des"_a, "a_w__des"_a, "j_w__des"_a)
      .def("SetYawDes", &LeeControl::SetYawDes,
           "yaw_des"_a, "yawdot_des"_a)
      .def("SetYawDotDes", &LeeControl::SetYawDotDes,
           "yawdot_des"_a)
      .def("SetAttitudeDes", &LeeControl::SetAttitudeDes,
           "w_q_b__des"_a, "omega_b__des"_a,
           "rpVelOnly"_a, "yawVelOnly"_a,
           "doSaturation"_a)
      .def("SetAttitudeAngAccelDes", &LeeControl::SetAttitudeAngAccelDes,
           "w_q_b__des"_a, "omega_b__des"_a, "alpha_b__des"_a)

// ----------------------------------------------------------------------
// Added lambdas
// ----------------------------------------------------------------------

      .def("GetModel", [](LeeControl &lee)
        { FloatingPoint mass; Matrix3 inertia;
          int r = lee.GetModel(&mass, &inertia);
          return std::make_tuple(
            mass,
            inertia,
            r);
      })

      .def("GetWorldParams", [](LeeControl &lee)
        { FloatingPoint gravityMag; FloatingPoint atmosphereDensity;
          int r = lee.GetWorldParams(&gravityMag, &atmosphereDensity);
          return std::make_tuple(
            gravityMag,
            atmosphereDensity,
            r);
      })

      .def("GetGains", [](LeeControl &lee)
        { Vector3 k_x; Vector3 k_v; Vector3 k_R; Vector3 k_omega;
          int r = lee.GetGains(&k_x, &k_v, &k_R, &k_omega);
          return std::make_tuple(
            k_x,
            k_v,
            k_R,
            k_omega,
            r);
      })

      .def("GetState", [](LeeControl &lee)
        { Vector3 x_w; Matrix3 w_R_b; Vector3 v_b; Vector3 omega_b; Vector3 a_b;
          int r = lee.GetState(&x_w, &w_R_b, &v_b, &omega_b, &a_b);
          return std::make_tuple(
            x_w,
            w_R_b,
            v_b,
            omega_b,
            a_b,
            r);
      })

      .def("GetDesired", [](LeeControl &lee) 
        { Vector3 x_w__des; Quaternion w_q_b__des;
           Vector3 v_b__des; Vector3 omega_b__des;
           int r = lee.GetDesired(&x_w__des, &w_q_b__des, &v_b__des, &omega_b__des);
           const Matrix3 w_R_b(w_q_b__des);
           const Vector3 w_euler_b = w_R_b.eulerAngles(2, 1, 0);
           Eigen::Vector4d w_quat_as_vec_b(w_q_b__des.x(),
            w_q_b__des.y(),
            w_q_b__des.z(),
            w_q_b__des.w());
           return std::make_tuple(x_w__des,
            w_euler_b,
            w_quat_as_vec_b,
            v_b__des,
            omega_b__des,
            r);
      })

      .def("GetAngAccelCommand", [](LeeControl &lee, bool rpVelOnly, bool yawVelOnly)
        { Vector3 alpha_b__comm;
          int r = lee.GetAngAccelCommand(&alpha_b__comm, rpVelOnly, yawVelOnly);
          return std::make_tuple(
            alpha_b__comm,
            r);
      })

      .def("GetAccelAngAccelCommand", [](LeeControl &lee)
        { Vector3 a_w__comm; Vector3 alpha_b__comm;
          int r = lee.GetAccelAngAccelCommand(&a_w__comm, &alpha_b__comm);
          return std::make_tuple(
            a_w__comm,
            alpha_b__comm,
            r);
      })

      .def("SetOdometry", [](LeeControl &lee, Vector3 &x_w, Eigen::Vector4d &w_q_b_vec, Vector3 &v_b, Vector3 &omega_b) 
        { Quaternion w_q_b(w_q_b_vec(3),
            w_q_b_vec(0),
            w_q_b_vec(1),
            w_q_b_vec(2));
          int r = lee.SetOdometry(x_w, w_q_b, v_b, omega_b);
          return r;
      })

      .def("SetAttitudeAngVel", [](LeeControl &lee, Eigen::Vector4d &w_q_b_vec, Vector3 &omega_b) 
        { Quaternion w_q_b(w_q_b_vec(3),
            w_q_b_vec(0),
            w_q_b_vec(1),
            w_q_b_vec(2));
          int r = lee.SetAttitudeAngVel(w_q_b, omega_b);
          return r;
      })

      .def("SetAttitudeDes", [](LeeControl &lee, Eigen::Vector4d &w_q_b_vec, Vector3 &omega_b__des,
                            bool rpVelOnly, bool yawVelOnly) 
        { Quaternion w_q_b__des(w_q_b_vec(3),
            w_q_b_vec(0),
            w_q_b_vec(1),
            w_q_b_vec(2));
          int r = lee.SetAttitudeDes(w_q_b__des, omega_b__des, rpVelOnly, yawVelOnly);
          return r;
      })

      .def("SetAttitudeAngAccelDes", [](LeeControl &lee, Eigen::Vector4d &w_q_b_vec, 
                                    Vector3 &omega_b__des, Vector3 &alpha_b__des) 
        { Quaternion w_q_b__des(w_q_b_vec(3),
            w_q_b_vec(0),
            w_q_b_vec(1),
            w_q_b_vec(2));
          int r = lee.SetAttitudeAngAccelDes(w_q_b__des, omega_b__des, alpha_b__des);
          return r;
      })

// ----------------------------------------------------------------------
// (formerly) Private helper functions
// ----------------------------------------------------------------------
      // added
      .def("so3Error", &LeeControl::so3Error,
            "e_R"_a, "e_omega"_a, "rpVelOnly"_a, "yawVelOnly"_a);

}