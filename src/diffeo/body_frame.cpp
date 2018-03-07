#include "../../include/quest_gnc/diffeo/body_frame.h"

#include "Eigen/Geometry"

namespace quest_gnc {
namespace multirotor {

BodyFrame::BodyFrame() {
    // TODO
}

BodyFrame::~BodyFrame() {
    // TODO
}

int BodyFrame::
  FromYawAccel(float yaw, const Eigen::Vector3d& accel,
               Eigen::Matrix3d& w_R_body) const {
    w_R_body.col(2) = accel;

    // could switch to normalize() if not using thrust_norm later
    const double thrust_norm = w_R_body.col(2).norm();
    w_R_body.col(2) /= thrust_norm;

    Eigen::Vector3d x_c(cos(yaw), sin(yaw), 0);
    w_R_body.col(1) = w_R_body.col(2).cross(x_c);
    //w_R_body.col(1).normalize();
    w_R_body.col(0) = w_R_body.col(1).cross(w_R_body.col(2));
    //w_R_body.col(0).normalize();

    return 0;
}

} // namespace multirotor
} // namespace quest_gnc
