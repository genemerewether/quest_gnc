#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_EST_IMU_INTEG_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_EST_IMU_INTEG_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/world_params.h"

#include "quest_gnc/utils/common.h"

namespace quest_gnc {
namespace estimation {

class ImuInteg {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuInteg();

    ImuInteg(WorldParams wParams);

    ~ImuInteg();

  // ----------------------------------------------------------------------
  // Parameter and model setters
  // ----------------------------------------------------------------------

    int SetWorldParams(WorldParams wParams);

  // ----------------------------------------------------------------------
  // Output getters
  // ----------------------------------------------------------------------

    int GetState(Vector3* x_w,
                 Quaternion* w_q_b,
                 Vector3* v_b,
                 Vector3* omega_b);

  // ----------------------------------------------------------------------
  // Input setters
  // ----------------------------------------------------------------------

    int AddImu(const Vector3& omega_b,
               const Vector3& a_b);

    int SetUpdate(FloatingPoint tValid,
                  const Vector3& x_w,
                  const Quaternion& w_q_b,
                  const Vector3& v_w,
                  const Vector3& wBias,
                  const Vector3& aBias);

    int SetUpdate(FloatingPoint tValid,
                  const Vector3& x_w,
                  const Quaternion& w_q_b,
                  const Vector3& v_w,
                  const Vector3& wBias,
                  const Vector3& aBias,
                  const Quaternion& b_q_g,
                  const Matrix3& aGyrInv,
                  const Matrix3& aAccInv);

  // ----------------------------------------------------------------------
  // Processing functions
  // ----------------------------------------------------------------------

    int PropagateState();

 private:
    // Parameters
    WorldParams wParams;

    // Stand-in for ring buffer of IMU samples
    bool newMeas;
    Vector3 omega_b__meas;
    Vector3 a_b__meas;

    // Odometry

    FloatingPoint tLastUpdate;

    Vector3 x_w;

    Matrix3 w_R_b;

    Vector3 v_b;

    Vector3 omega_b;

    // Sensor Parameters

    // Accelerometer gyro rotation matrix estimate
    // (b = body = accelerometer, g = gyro)
    Matrix3 b_R_g;

    // Gyro bias estimate (rad/s)
    Vector3 wBias;

    // Accelerometer bias estimate (m/s^2)
    Vector3 aBias;

    // Inverse of accelerometer scale and non-orthogonality estimate
    Matrix3 aGyrInv;

    // Inverse of gyro scale and non-orthogonality estimate
    Matrix3 aAccInv;

}; // class ImuInteg NOLINT()

} // namespace estimation NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_EST_IMU_INTEG_H_
