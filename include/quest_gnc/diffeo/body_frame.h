#ifndef QUEST_GNC_DIFFEO_BODY_FRAME_H
#define QUEST_GNC_DIFFEO_BODY_FRAME_H

#include <Eigen/Eigen>

namespace quest_gnc {

class BodyFrame {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BodyFrame();

    ~BodyFrame();
}; // class BodyFrame

// body frame from thrust vector and 4th flat output

} // namespace quest_gnc

#endif // #ifndef QUEST_GNC_DIFFEO_BODY_FRAME_H
