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

#include "quest_gnc/utils/common.h"
#include "quest_gnc/utils/world_params.h"
#include "quest_gnc/utils/rigidbody_model.h"
#include "quest_gnc/utils/multirotor_model.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using namespace quest_gnc;
using namespace quest_gnc::multirotor;
using namespace pybind11::literals; // NOLINT()

void common_bind(py::module &m) { // NOLINT()
  py::class_<ImuSample>(m, "ImuSample")
      .def(py::init<>())

      .def_readwrite("t", &ImuSample::t)
      .def_readwrite("omega_b", &ImuSample::omega_b)
      .def_readwrite("a_b", &ImuSample::a_b);

  py::class_<WorldParams>(m, "WorldParams")
      .def(py::init<>())

      .def_readwrite("gravityMag", &WorldParams::gravityMag)
      .def_readwrite("atmosphereDensity", &WorldParams::atmosphereDensity);

  py::class_<RigidBodyModel>(m, "RigidBodyModel")
      .def(py::init<>())

      .def_readwrite("mass", &RigidBodyModel::mass)
      .def_readwrite("Ixx", &RigidBodyModel::Ixx)
      .def_readwrite("Iyy", &RigidBodyModel::Iyy)
      .def_readwrite("Izz", &RigidBodyModel::Izz)
      .def_readwrite("Ixy", &RigidBodyModel::Ixy)
      .def_readwrite("Ixz", &RigidBodyModel::Ixz)
      .def_readwrite("Iyz", &RigidBodyModel::Iyz);

  py::class_<MultirotorModel>(m, "MultirotorModel")
      .def(py::init<>())

      .def_readwrite("rigidBody", &MultirotorModel::rigidBody);
}
