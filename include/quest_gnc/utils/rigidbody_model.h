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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_RIGIDBODY_MODEL_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_RIGIDBODY_MODEL_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {

struct RigidBodyModel {
    FloatingPoint mass;

    FloatingPoint Ixx;
    FloatingPoint Iyy;
    FloatingPoint Izz;

    FloatingPoint Ixy;
    FloatingPoint Ixz;
    FloatingPoint Iyz;
}; // struct RigidBodyModel NOLINT()

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_RIGIDBODY_MODEL_H_
