#ifndef QUEST_GNC_DIFFEO_BODY_FRAME_H
#define QUEST_GNC_DIFFEO_BODY_FRAME_H

#include <Eigen/Eigen>

namespace quest_gnc {
namespace multirotor {

class BodyFrame {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BodyFrame();

    ~BodyFrame();

    int FromYawAccel(float yaw, const Eigen::Vector3d& accel,
                     Eigen::Matrix3d& w_R_body) const;

    // TODO(mereweth) - support 2nd angle, q3, closest axis methods

    // TODO(mereweth) - convenience methods for setting attitude with acceleration waypoint

    // TODO(mereweth) - put angular rate saturation here?

  private:
    // parameters object
}; // class BodyFrame

} // namespace multirotor
} // namespace quest_gnc

#endif // #ifndef QUEST_GNC_DIFFEO_BODY_FRAME_H
