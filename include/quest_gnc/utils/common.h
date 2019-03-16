// ======================================================================
// \author mereweth
//
// \copyright
// Copyright 2009-2018, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
//
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ======================================================================

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_COMMON_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_COMMON_H_

#include <Eigen/Eigen>

#include <math.h>

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

#ifndef COMPILE_TIME_ASSERT
#define COMPILE_TIME_ASSERT( condition, name )\
  do { \
    enum { assert_failed_ ## name = 1/(condition) }; \
  } while(0)
#endif

#ifndef M_PI
#ifdef BUILD_DSPAL
#define M_PI 3.14159265358979323846
#endif
#endif

namespace quest_gnc {

#ifndef QUEST_GNC_FLOAT_TYPE
typedef double FloatingPoint;

#else
typedef QUEST_GNC_FLOAT_TYPE FloatingPoint;
#endif

typedef Eigen::Matrix<FloatingPoint, 3, 1> Vector3;
typedef Eigen::Matrix<FloatingPoint, 3, 3> Matrix3;
typedef Eigen::Matrix<FloatingPoint, 4, 4> Matrix4;
typedef Eigen::Quaternion<FloatingPoint> Quaternion;
typedef Eigen::AngleAxis<FloatingPoint> AngleAxis;

struct ImuSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double t;
    Vector3 omega_b;
    Vector3 a_b;
};

struct MagSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double t;
    Vector3 m_b;
};

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_COMMON_H_
