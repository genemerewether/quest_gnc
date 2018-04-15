#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_MULTIROTOR_MODEL_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_MULTIROTOR_MODEL_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {
namespace multirotor {

struct MultirotorModel {
    FloatingPoint mass;

    FloatingPoint Ixx;
    FloatingPoint Iyy;
    FloatingPoint Izz;

    FloatingPoint Ixy;
    FloatingPoint Ixz;
    FloatingPoint Iyz;
}; // struct MultirotorModel NOLINT()

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_MULTIROTOR_MODEL_H_
