#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_MULTIROTOR_MODEL_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_MULTIROTOR_MODEL_H_

#include <Eigen/Eigen>

namespace quest_gnc {
namespace multirotor {

struct MultirotorModel {
    double mass;

    double Ixx;
    double Iyy;
    double Izz;

    double Ixy;
    double Ixz;
    double Iyz;
}; // struct MultirotorModel NOLINT()

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_MULTIROTOR_MODEL_H_
