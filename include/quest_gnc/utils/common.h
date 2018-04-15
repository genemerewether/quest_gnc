#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_COMMON_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_COMMON_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {

#ifndef QUEST_GNC_FLOAT_TYPE
typedef double FloatingPoint;

#else
typedef QUEST_GNC_FLOAT_TYPE FloatingPoint;
#endif

typedef Eigen::Matrix<FloatingPoint, 3, 1> Vector3;
typedef Eigen::Matrix<FloatingPoint, 3, 3> Matrix3;
typedef Eigen::Quaternion<FloatingPoint> Quaternion;

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_COMMON_H_
