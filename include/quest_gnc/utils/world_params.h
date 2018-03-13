#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_WORLD_PARAMS_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_WORLD_PARAMS_H_

#include <Eigen/Eigen>

namespace quest_gnc {

struct WorldParams {
     // 9.80665 m/s2
    double gravityMag;

     // 1.2 kg/m3
    double atmosphereDensity;
}; // struct WorldParams NOLINT()

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_WORLD_PARAMS_H_
