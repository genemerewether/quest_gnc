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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_EST_ATT_FILTER_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_EST_ATT_FILTER_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/world_params.h"
#include "quest_gnc/utils/common.h"
#include "quest_gnc/utils/ringbuffer.h"

namespace quest_gnc {
namespace estimation {

class AttFilter {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ringbuffer<ImuSample, 1000> ImuBuffer;
    typedef ringbuffer<MagSample, 100> MagBuffer;

    AttFilter();

    AttFilter(WorldParams wParams);

    ~AttFilter();

  // ----------------------------------------------------------------------
  // Parameter and model setters
  // ----------------------------------------------------------------------

    int Reinit();
    
    int SetWorldParams(WorldParams wParams);
    
    int SetTimeStep(FloatingPoint dt);

    int SetSteadyStateThresh(FloatingPoint accelThresh,
                             FloatingPoint omega_b__deltaThresh,
                             FloatingPoint omega_b__thresh);

    int SetAccelGain(FloatingPoint gain);

    int SetBiasAlpha(FloatingPoint biasAlpha);

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

    int AddImu(const ImuSample& imu);

    int AddMag(const MagSample& mag);

    int SetUpdate(double tValid,
                  const Vector3& x_w,
                  const Quaternion& w_q_b,
                  const Vector3& v_w,
                  const Vector3& wBias,
                  const Vector3& aBias);

    int SetUpdate(double tValid,
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

    FloatingPoint accelThresh;

    FloatingPoint omega_b__deltaThresh;

    FloatingPoint omega_b__thresh;

    FloatingPoint biasAlpha;

    FloatingPoint accelGain;

    bool initialized;

    // Nominal dt
    FloatingPoint dt;

    ImuBuffer imuBuf;

    Vector3 omega_b__prev;

    MagBuffer magBuf;

    double tLastIntegrated;
    
    // Odometry

    double tLastUpdate;

    Vector3 x_w;

    Quaternion b_q_w;

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

    // TODO (mereweth) - what is observable by high-level filter for magnetometer?

}; // class AttFilter NOLINT()

} // namespace estimation NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_EST_ATT_FILTER_H_
