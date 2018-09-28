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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_RATES_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_RATES_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {
namespace multirotor {

class Rates {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Rates();

    ~Rates();

    int GetPDotQDot(FloatingPoint thrust,
                    const Vector3& jerk_w,
                    const Vector3& snap_w,
                    const Matrix3& w_R_b,
                    const Vector3& omega_b,
                    FloatingPoint* pDot,
                    FloatingPoint* qDot) const;

    int GetPQ(FloatingPoint thrust,
              const Vector3& jerk_w,
              const Matrix3& w_R_b,
              FloatingPoint* p,
              FloatingPoint* q) const;

    int ProjectYawDerivToBody(FloatingPoint yaw_deriv,
                              const Vector3& zBody_w,
                              FloatingPoint* body_z_deriv) const;

 private:
    // parameters object - warning tolerances, physical parameters
}; // class Rates NOLINT()

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_RATES_H_
