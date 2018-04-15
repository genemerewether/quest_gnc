#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_WORLD_PARAMS_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_WORLD_PARAMS_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {

struct WorldParams {
     // 9.80665 m/s2
    FloatingPoint gravityMag;

     // 1.2 kg/m3
    FloatingPoint atmosphereDensity;
}; // struct WorldParams NOLINT()

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_WORLD_PARAMS_H_
