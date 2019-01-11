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


#include "quest_gnc/est/imu_integ.h"

#include "quest_gnc/utils/so3.h"

#include "Eigen/Geometry"

#include <math.h>

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

//#define DEBUG_PRINT(x,...)

#ifndef DEBUG_PRINT
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
#endif

namespace quest_gnc {
namespace estimation {

ImuInteg::ImuInteg() :
    wParams(),
    dt(0.0),
    newMeas(false), omega_b__meas(0, 0, 0), a_b__meas(0, 0, 0),
    tLastUpdate(0),
    x_w(0, 0, 0), w_R_b(Matrix3::Identity()),
    v_b(0, 0, 0), omega_b(0, 0, 0),
    b_R_g(Matrix3::Identity()),
    wBias(0, 0, 0), aBias(0, 0, 0),
    aGyrInv(Matrix3::Identity()), aAccInv(Matrix3::Identity()) {
}

ImuInteg::~ImuInteg() {
    // TODO(mereweth)
}

// ----------------------------------------------------------------------
// Parameter, model, and gain setters
// ----------------------------------------------------------------------

int ImuInteg::
  SetWorldParams(WorldParams wParams) {
    this->wParams = wParams;

    return 0;
}

int ImuInteg::
  SetTimeStep(FloatingPoint dt) {
    if (dt < 0.0) {
        return -1;
    }
  
    this->dt = dt;

    return 0;
}
  
// ----------------------------------------------------------------------
// Output getters
// ----------------------------------------------------------------------

int ImuInteg::
  GetState(Vector3* x_w,
           Quaternion* w_q_b,
           Vector3* v_b,
           Vector3* omega_b) {
    FW_ASSERT(x_w);
    FW_ASSERT(w_q_b);
    FW_ASSERT(v_b);
    FW_ASSERT(omega_b);

    *x_w = this->x_w;
    *w_q_b = this->w_R_b;
    *v_b = this->v_b;
    *omega_b = this->omega_b;

    return 0;
}

// ----------------------------------------------------------------------
// Input setters
// ----------------------------------------------------------------------

int ImuInteg::
  AddImu(const Vector3& omega_b,
         const Vector3& a_b) {
    /* TODO(mereweth) - don't add IMU data from before time of latest state
     * update
     */

    /* TODO(mereweth) - error if overflowing ring buffer
     */

    // TODO(mereweth) - add IMU sample to ring buffer

    int status = 0;
    // TODO(mereweth) - return codes
    if (this->newMeas) {  status = -1;  }
    this->newMeas = true;
    this->omega_b__meas = omega_b;
    this->a_b__meas = a_b;

    return status;
}

int ImuInteg::
  SetUpdate(FloatingPoint tValid,
            const Vector3& x_w,
            const Quaternion& w_q_b,
            const Vector3& v_w,
            const Vector3& wBias,
            const Vector3& aBias) {
    /* TODO(mereweth) - don't allow state updates from before time of latest state
     * update
     */

    this->tLastUpdate = tValid;
    this->x_w = x_w;
    this->w_R_b = w_q_b.toRotationMatrix();
    this->v_b = this->w_R_b.transpose() * v_w;
    this->wBias = wBias;
    this->aBias = aBias;

    return 0;
}

int ImuInteg::
  SetUpdate(FloatingPoint tValid,
            const Vector3& x_w,
            const Quaternion& w_q_b,
            const Vector3& v_w,
            const Vector3& wBias,
            const Vector3& aBias,
            const Quaternion& b_q_g,
            const Matrix3& aGyrInv,
            const Matrix3& aAccInv) {
    /* TODO(mereweth) - don't allow state updates from before time of latest state
     * update
     */

    this->b_R_g = b_q_g.toRotationMatrix();
    this->aGyrInv = aGyrInv;
    this->aAccInv = aAccInv;

    // TODO(mereweth) - use return value of setUpdate
    return SetUpdate(tValid, x_w, w_q_b, v_w, wBias, aBias);
}

// ----------------------------------------------------------------------
// Processing functions
// ----------------------------------------------------------------------

int ImuInteg::
  PropagateState() {
    /* NOTE(mereweth) - can only discard IMU data once state update with time
     * stamp after the IMU data stamp has been received
     */

    /* TODO(mereweth) - check for gaps in IMU data
     */

    // start at head of ring buffer, go to tail
    // for each IMU sample:
        // peek at sample
        // if older than tLastUpdate, discard
        // optionally:
            // add IMU sample to filter
            // pull latest filter element
        // optionally:
            // bump detection (based on vehicle model)
        // optionally:
            // apply rotation matrix, scale, and non-orthogonality corrections
        // offset by bias
        // TODO(mereweth) - check dt with exponential filter

    // TODO(mereweth) - return codes
    if (!this->newMeas) {
        return -1;
    }

    // -------------------------- Rotation kinematics --------------------------

    this->omega_b = this->omega_b__meas - wBias;
    const Vector3 omega_b__dt = dt * this->omega_b;
    Matrix3 expMap;
    expMap3(omega_b__dt, &expMap);
    const Matrix3 w_R_b__temp = this->w_R_b * expMap;

    // -------------------------- Position kinematics --------------------------

    const Vector3 a_b__temp = this->a_b__meas - aBias - this->w_R_b.transpose()
                              * Vector3(0, 0, wParams.gravityMag);

    this->v_b += a_b__temp * dt;
    this->x_w += this->w_R_b * this->v_b * dt;

    this->w_R_b = w_R_b__temp;

    return 0;
}

} // namespace estimation NOLINT()
} // namespace quest_gnc NOLINT()
