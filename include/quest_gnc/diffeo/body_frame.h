#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_BODY_FRAME_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_BODY_FRAME_H_

#include <Eigen/Eigen>

namespace quest_gnc {
namespace multirotor {

class BodyFrame {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BodyFrame();

    ~BodyFrame();

    int FromYawAccel(float yaw, const Eigen::Vector3d& accel_w,
                     Eigen::Matrix3d* w_R_b) const;

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
