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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_BODY_FRAME_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_BODY_FRAME_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {
namespace multirotor {

class BodyFrame {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BodyFrame();

    ~BodyFrame();

    int FromYawAccel(float yaw, const Vector3& accel_w,
                     Matrix3* w_R_b) const;

    // TODO(mereweth) - support 2nd angle, q3, closest axis methods

    /* TODO(mereweth) - convenience methods for setting attitude with
     * acceleration waypoint
     */

    // TODO(mereweth) - put angular rate saturation here?

 private:
    // parameters object - warning tolerances, physical parameters
}; // class BodyFrame NOLINT()

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_BODY_FRAME_H_
