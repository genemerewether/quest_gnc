#include "quest_gnc/diffeo/body_frame.h"

#include "Eigen/Geometry"

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

namespace quest_gnc {
namespace multirotor {

BodyFrame::BodyFrame() {
    // TODO(mereweth)
}

BodyFrame::~BodyFrame() {
    // TODO(mereweth)
}

int BodyFrame::
  FromYawAccel(float yaw, const Vector3& accel_w,
               Matrix3* w_R_b) const {
    FW_ASSERT(w_R_b);
    w_R_b->col(2) = accel_w;

    // could switch to normalize() if not using thrust_norm later
    const FloatingPoint thrust_norm = w_R_b->col(2).norm();
    w_R_b->col(2) /= thrust_norm;

    Vector3 x_c(cos(yaw), sin(yaw), 0);
    w_R_b->col(1) = w_R_b->col(2).cross(x_c);
    // w_R_b->col(1).normalize();
    w_R_b->col(0) = w_R_b->col(1).cross(w_R_b->col(2));
    // w_R_b->col(0).normalize();

    return 0;
}

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()
