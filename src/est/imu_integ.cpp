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

//#define DEBUG_PRINT(x,...)

#ifndef DEBUG_PRINT
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
#endif

namespace quest_gnc {
namespace estimation {

ImuInteg::ImuInteg() :
    wParams(),
    dt(0.0),
    imuBuf(),
    tLastIntegrated(0.0),
    tLast(0.0),
    tLastUpdate(0.0),
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
           Vector3* omega_b,
           Vector3* a_b) {
    FW_ASSERT(x_w);
    FW_ASSERT(w_q_b);
    FW_ASSERT(v_b);
    FW_ASSERT(omega_b);
    FW_ASSERT(a_b);

    *x_w = this->x_w;
    *w_q_b = this->w_R_b;
    w_q_b->normalize();
    *v_b = this->v_b;
    *omega_b = this->omega_b;
    *a_b = this->a_b;

    return 0;
}

// ----------------------------------------------------------------------
// Input setters
// ----------------------------------------------------------------------

int ImuInteg::
  AddImu(const ImuSample& imu) {
    // could only happen with out-of-order IMU data
    if (imu.t <= tLastUpdate) {
        return -1;
    }
    // could only happen with out-of-order IMU data
    if (this->imuBuf.size() &&
        (imu.t <= this->imuBuf.getLastIn()->t)) {
        return -2;
    }

    return this->imuBuf.queue(&imu);
}

int ImuInteg::
  SetUpdate(double tValid,
            const Vector3& x_w,
            const Quaternion& w_q_b,
            const Vector3& v_b,
            const Vector3& wBias,
            const Vector3& aBias) {
    // could only happen with out-of-order state update data
    if (tValid <= tLastUpdate) {
        return -1;
    }

    // protect against old state update that IMU buffer doesn't cover
    if (this->imuBuf.size() &&
        (tValid < this->imuBuf.getFirstIn()->t)) {
        return -2;
    }

    // TODO(mereweth) - what to do if we have no IMU samples?
    // should only be possible with hardware failure

    // TODO(mereweth) - check for state update after most recent IMU sample?
    // should only be possible in case of bad clock sync
    
    this->tLast = this->imuBuf.getFirstIn()->t;

    this->tLastUpdate = tValid;
    this->tLastIntegrated = tValid;
    this->x_w = x_w;
    this->w_R_b = w_q_b.normalized().toRotationMatrix();
    this->v_b = v_b;
    this->wBias = wBias;
    this->aBias = aBias;

    return 0;
}

int ImuInteg::
  SetUpdate(double tValid,
            const Vector3& x_w,
            const Quaternion& w_q_b,
            const Vector3& v_b,
            const Vector3& wBias,
            const Vector3& aBias,
            const Quaternion& b_q_g,
            const Matrix3& aGyrInv,
            const Matrix3& aAccInv) {
    int stat = SetUpdate(tValid, x_w, w_q_b, v_b, wBias, aBias);
    if (stat) {
        return stat;
    }

    this->b_R_g = b_q_g.normalized().toRotationMatrix();
    this->aGyrInv = aGyrInv;
    this->aAccInv = aAccInv;

    return 0;
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

    // discard IMU samples from before last update
    while (this->imuBuf.size() &&
           (this->tLastUpdate > this->imuBuf.getFirstIn()->t)) {
        quest_gnc::ImuSample imu;
        this->imuBuf.dequeue(&imu);
        this->tLast = imu.t;
    }

    // NOTE(mereweth) - need to keep all IMU samples since last update
    // in case another update from right after it comes in
    for (int i = 0; i < this->imuBuf.size(); i++) {
        const quest_gnc::ImuSample* imu = this->imuBuf.get(i);
        FW_ASSERT(imu != NULL);

        // TODO(mereweth) - parameter for small time delta - check for duplicate IMU
        if (imu->t < this->tLast + 1e-6) {
            DEBUG_PRINT("dup imu at %f in imu_integ\n", imu->t);
            continue;
        }
        
        FloatingPoint _dt = imu->t - this->tLast;
        if (this->tLastUpdate > this->tLast) { // update came halfway through imu sampling period
            _dt = imu->t - this->tLastUpdate;
        }
        if (fabs(this->dt - _dt) > this->dt * 0.01) {
            // NOTE(mereweth) - evr that dt was more than 1% off
            DEBUG_PRINT("imu_integ nominal dt %f, actual %f\n",
                        this->dt, _dt);
            _dt = this->dt;
        }
        this->tLast = imu.t;
        
        this->tLastIntegrated = imu->t;
          
        // -------------------------- Rotation kinematics --------------------------

        this->omega_b = imu->omega_b - this->wBias;
        const Vector3 omega_b__dt = this->dt * this->omega_b;
        Matrix3 expMap;
        expMap3(omega_b__dt, &expMap);
        const Matrix3 w_R_b__temp = this->w_R_b * expMap;

        // -------------------------- Position kinematics --------------------------

        // store unbiased linear acceleration for odometry
        this->a_b = imu->a_b - this->aBias - this->w_R_b.transpose()
                    * Vector3(0, 0, this->wParams.gravityMag);

        this->v_b += this->a_b * this->dt;
        this->x_w += this->w_R_b * this->v_b * this->dt;

        this->w_R_b = w_R_b__temp;
    }

    return 0;
}

} // namespace estimation NOLINT()
} // namespace quest_gnc NOLINT()
