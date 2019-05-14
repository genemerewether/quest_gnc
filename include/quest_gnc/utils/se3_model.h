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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_SE3_MODEL_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_SE3_MODEL_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/rigidbody_model.h"

#include "quest_gnc/utils/common.h"

namespace quest_gnc {

struct Se3Model {
    RigidBodyModel rigidBody;
    // static force in world z from e.g. buoyancy (+) / tether (-)
    FloatingPoint force_z;
}; // struct Se3Model NOLINT()

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_SE3_MODEL_H_
