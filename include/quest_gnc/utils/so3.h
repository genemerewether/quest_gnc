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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_SO3_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_SO3_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

namespace quest_gnc {

inline void hat3(const Vector3& v, Matrix3* m) {
    FW_ASSERT(m);

    *m << 0, -v.z(), v.y(),
          v.z(), 0, -v.x(),
          -v.y(), v.x(), 0;
}

inline int vee3(Vector3* v, const Matrix3& m) {
    FW_ASSERT(v);

    *v = Vector3(m(2, 1), m(0, 2), m(1, 0));

    // TODO(mereweth) - tolerance?
    if ((fabs(m(2, 1) + m(1, 2)) > 1e-4) ||
        (fabs(m(0, 2) + m(2, 0)) > 1e-4) ||
        (fabs(m(1, 0) + m(0, 1)) > 1e-4)) {
        return -1;
    }
    else {
        return 0;
    }
}

inline int expMap3(const Vector3& v, Matrix3* m) {
    FW_ASSERT(m);

    Matrix3 v_hat;
    hat3(v, &v_hat);
    const FloatingPoint theta = v.dot(v);
    const FloatingPoint thetaSquared = theta * theta;

    /* NOTE(mereweth) - check for small angle rotation about angular velocity
     * vector; use Taylor expansion for trig terms
     */
    FloatingPoint sinThetaByTheta = 0.0;
    FloatingPoint oneMinusCosThetaByThetaSquared = 0.0;
    // TODO(mereweth) - use parameter object for threshold on theta
    if (theta < 0.01) {
        sinThetaByTheta = 1.0 - thetaSquared / 6.0
                          + thetaSquared * thetaSquared / 120.0;
        oneMinusCosThetaByThetaSquared = 0.5 - thetaSquared / 24.0
                                         + thetaSquared * thetaSquared / 720.0;
    }
    else {
        sinThetaByTheta = sin(theta) / theta;
        oneMinusCosThetaByThetaSquared = (1.0 - cos(theta)) / thetaSquared;
    }

    *m = Matrix3::Identity() + sinThetaByTheta * v_hat
         + oneMinusCosThetaByThetaSquared * v_hat * v_hat;

    return 0;
}

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_SO3_H_