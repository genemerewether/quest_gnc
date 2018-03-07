#include "../../include/quest_gnc/diffeo/rates.h"

#include "Eigen/Geometry"

namespace quest_gnc {
namespace multirotor {

Rates::Rates() {
    // TODO
}

Rates::~Rates() {
    // TODO
}

int Rates::
  GetPDotQDot(double thrust,
              const Eigen::Vector3d& jerk,
              const Eigen::Vector3d& snap,
              const Eigen::Matrix3d& w_R_body,
              const Eigen::Vector3d& omega,
              double& pDot,
              double& qDot) const {
    Eigen::Vector3d h_alpha = snap - w_R_body.col(2).dot(snap) * w_R_body.col(3)
                              + thrust * w_R_body.col(2).dot(
                                  omega.cross(omega.cross(w_R_body.col(2)))) * w_R_body.col(2)
                              - thrust * omega.cross(omega.cross(w_R_body.col(2)))
                              - 2 * omega.cross(w_R_body.col(2).dot(jerk) * w_R_body.col(2));

    h_alpha /= thrust;

    pDot = -1.0 * h_alpha.dot(w_R_body.col(1));
    qDot = h_alpha.dot(w_R_body.col(0));

    return 0;
}

int Rates::
  ProjectYawDerivToBody(double yaw_deriv,
                        const Eigen::Vector3d& zBody_w,
                        double& body_z_deriv) const {
    body_z_deriv = yaw_deriv * Eigen::Vector3d::UnitZ().dot(zBody_w);

    // TODO(mereweth) - return warning code if zBody_w perpendicular to z_w
    return 0;
}

int Rates::
  GetPQ(double thrust,
        const Eigen::Vector3d& jerk,
        const Eigen::Matrix3d& w_R_body,
        double& p,
        double& q) const {

    Eigen::Vector3d h_omega = jerk - w_R_body.col(2).dot(jerk) * w_R_body.col(2);
    h_omega /= thrust;

    p = -1.0 * h_omega.dot(w_R_body.col(1));
    q = h_omega.dot(w_R_body.col(0));

    return 0;
}

} // namespace multirotor
} // namespace quest_gnc
