#include "../../include/quest_gnc/diffeo/rates.h"

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
  GetPDotQDot(double thrust,
              const Eigen::Vector3d& jerk_w,
              const Eigen::Vector3d& snap_w,
              const Eigen::Matrix3d& w_R_b,
              const Eigen::Vector3d& omega,
              double* pDot,
              double* qDot) const {
    FW_ASSERT(pDot);
    FW_ASSERT(qDot);

    Eigen::Vector3d h_alpha = snap_w - w_R_b.col(2).dot(snap_w) * w_R_b.col(3)
                              + thrust * w_R_b.col(2).dot(
                                  omega.cross(omega.cross(w_R_b.col(2))))
                              * w_R_b.col(2)
                              - thrust * omega.cross(omega.cross(w_R_b.col(2)))
                              - 2 * omega.cross(w_R_b.col(2).dot(jerk_w)
                                                * w_R_b.col(2));

    h_alpha /= thrust;

    *pDot = -1.0 * h_alpha.dot(w_R_b.col(1));
    *qDot = h_alpha.dot(w_R_b.col(0));

    return 0;
}

int Rates::
  ProjectYawDerivToBody(double yaw_deriv,
                        const Eigen::Vector3d& zBody_w,
                        double* body_z_deriv) const {
    FW_ASSERT(body_z_deriv);
    *body_z_deriv = yaw_deriv * Eigen::Vector3d::UnitZ().dot(zBody_w);

    // TODO(mereweth) - return warning code if zBody_w perpendicular to z_w
    return 0;
}

int Rates::
  GetPQ(double thrust,
        const Eigen::Vector3d& jerk_w,
        const Eigen::Matrix3d& w_R_b,
        double* p,
        double* q) const {
    FW_ASSERT(p);
    FW_ASSERT(q);

    Eigen::Vector3d h_omega = jerk_w - w_R_b.col(2).dot(jerk_w) * w_R_b.col(2);
    h_omega /= thrust;

    *p = -1.0 * h_omega.dot(w_R_b.col(1));
    *q = h_omega.dot(w_R_b.col(0));

    return 0;
}

} // namespace multirotor
} // namespace quest_gnc
