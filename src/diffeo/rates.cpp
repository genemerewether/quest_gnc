#include "quest_gnc/diffeo/rates.h"

#include "Eigen/Geometry"

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

namespace quest_gnc {
namespace multirotor {

Rates::Rates() {
    // TODO(mereweth)
}

Rates::~Rates() {
    // TODO(mereweth)
}

int Rates::
  GetPDotQDot(FloatingPoint thrust,
              const Vector3& jerk_w,
              const Vector3& snap_w,
              const Matrix3& w_R_b,
              const Vector3& omega,
              FloatingPoint* pDot,
              FloatingPoint* qDot) const {
    FW_ASSERT(pDot);
    FW_ASSERT(qDot);

    Vector3 h_alpha = snap_w - w_R_b.col(2).dot(snap_w) * w_R_b.col(3)
                              + thrust * w_R_b.col(2).dot(
                                  omega.cross(omega.cross(w_R_b.col(2))))
                              * w_R_b.col(2)
                              - thrust * omega.cross(omega.cross(w_R_b.col(2)))
                              - 2 * omega.cross(w_R_b.col(2).dot(jerk_w)
                                                * w_R_b.col(2));

    h_alpha /= thrust;

    *pDot = -1.0f * h_alpha.dot(w_R_b.col(1));
    *qDot = h_alpha.dot(w_R_b.col(0));

    return 0;
}

int Rates::
  ProjectYawDerivToBody(FloatingPoint yaw_deriv,
                        const Vector3& zBody_w,
                        FloatingPoint* body_z_deriv) const {
    FW_ASSERT(body_z_deriv);
    *body_z_deriv = yaw_deriv * Vector3::UnitZ().dot(zBody_w);

    // TODO(mereweth) - return warning code if zBody_w perpendicular to z_w
    return 0;
}

int Rates::
  GetPQ(FloatingPoint thrust,
        const Vector3& jerk_w,
        const Matrix3& w_R_b,
        FloatingPoint* p,
        FloatingPoint* q) const {
    FW_ASSERT(p);
    FW_ASSERT(q);

    Vector3 h_omega = jerk_w - w_R_b.col(2).dot(jerk_w) * w_R_b.col(2);
    h_omega /= thrust;

    *p = -1.0f * h_omega.dot(w_R_b.col(1));
    *q = h_omega.dot(w_R_b.col(0));

    return 0;
}

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()
