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


#include "quest_gnc/est/att_filter.h"

#include "quest_gnc/utils/so3.h"

#include "Eigen/Geometry"

#include <math.h>

#include <stdio.h>
#define DEBUG_PRINT(x,...)

#ifndef DEBUG_PRINT
#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif
#endif

namespace quest_gnc {
namespace estimation {

AttFilter::AttFilter() :
    wParams(),
    accelThresh(0.0),
    omega_b__deltaThresh(0.0),
    omega_b__thresh(0.0),
    biasAlpha(0.0),
    accelGain(0.0),
    initialized(false),
    dt(0.0),
    imuBuf(),
    omega_b__prev(),
    magBuf(),
    tLastIntegrated(0.0),
    tLastUpdate(0.0),
    x_w(0, 0, 0),
    b_q_w(Quaternion::Identity()),
    v_b(0, 0, 0), omega_b(0, 0, 0),
    b_R_g(Matrix3::Identity()),
    wBias(0, 0, 0), aBias(0, 0, 0),
    aGyrInv(Matrix3::Identity()), aAccInv(Matrix3::Identity()) {
}

AttFilter::~AttFilter() {
    // TODO(mereweth)
}

// ----------------------------------------------------------------------
// Parameter, model, and gain setters
// ----------------------------------------------------------------------

int AttFilter::
  Reinit() {
    this->initialized = false;

    this->omega_b__prev << 0.0f, 0.0f, 0.0f;
    
    this->x_w << 0.0f, 0.0f, 0.0f;
    this->b_q_w = Quaternion::Identity();
    this->v_b << 0.0f, 0.0f, 0.0f;
    this->omega_b << 0.0f, 0.0f, 0.0f;

    this->wBias << 0.0f, 0.0f, 0.0f;

    DEBUG_PRINT("reinit att_filter\n");
  
    return 0;
}
  
int AttFilter::
  SetWorldParams(WorldParams wParams) {
    this->wParams = wParams;

    return 0;
}

int AttFilter::
  SetTimeStep(FloatingPoint dt) {
    if (dt < 0.0) {
        return -1;
    }
  
    this->dt = dt;

    return 0;
}

int AttFilter::
  SetSteadyStateThresh(FloatingPoint accelThresh,
                       FloatingPoint omega_b__deltaThresh,
                       FloatingPoint omega_b__thresh) {
    if ((accelThresh < 0.0)          ||
        (omega_b__deltaThresh < 0.0) || 
        (omega_b__thresh < 0.0))      {
        return -1;
    }

    this->accelThresh = accelThresh;
    this->omega_b__deltaThresh = omega_b__deltaThresh;
    this->omega_b__thresh = omega_b__thresh;

    return 0;
}

int AttFilter::
  SetAccelGain(FloatingPoint gain) {
    if ((gain < 0) || (gain > 1.0)) {
        return -1;
    }

    this->accelGain = gain;

    return 0;
}

int AttFilter::
  SetBiasAlpha(FloatingPoint biasAlpha) {
    if ((biasAlpha < 0) || (biasAlpha > 1.0)) {
        return -1;
    }

    this->biasAlpha = biasAlpha;

    return 0;
}
  
// ----------------------------------------------------------------------
// Output getters
// ----------------------------------------------------------------------

int AttFilter::
  GetState(Vector3* x_w,
           Quaternion* w_q_b,
           Vector3* v_b,
           Vector3* omega_b) {
    FW_ASSERT(x_w);
    FW_ASSERT(w_q_b);
    FW_ASSERT(v_b);
    FW_ASSERT(omega_b);

    *x_w = this->x_w;
    this->b_q_w.normalize();
    *w_q_b = this->b_q_w.conjugate();
    *v_b = this->v_b;
    *omega_b = this->omega_b;
    
    //DEBUG_PRINT("GetState att_filter\n");

    return 0;
}

// ----------------------------------------------------------------------
// Input setters
// ----------------------------------------------------------------------

int AttFilter::
  AddImu(const ImuSample& imu) {
    // could only happen with out-of-order IMU data
    if (imu.t < tLastUpdate) {
        return -1;
    }
    // could only happen with out-of-order IMU data
    if (this->imuBuf.size() &&
        (imu.t < this->imuBuf.getLastIn()->t)) {
        return -2;
    }
    
    //DEBUG_PRINT("AddImu att_filter\n");

    return this->imuBuf.queue(&imu);
}

int AttFilter::
  AddMag(const MagSample& mag) {
    // could only happen with out-of-order MAG data
    if (mag.t < tLastUpdate) {
        return -1;
    }
    // could only happen with out-of-order MAG data
    if (this->magBuf.size() &&
        (mag.t < this->magBuf.getLastIn()->t)) {
        return -2;
    }

    return this->magBuf.queue(&mag);
}

int AttFilter::
  SetUpdate(double tValid,
            const Vector3& x_w,
            const Quaternion& w_q_b,
            const Vector3& v_w,
            const Vector3& wBias,
            const Vector3& aBias) {
    // could only happen with out-of-order state update data
    if (tValid < tLastUpdate) {
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

    this->tLastUpdate = tValid;
    this->tLastIntegrated = tValid;
    this->x_w = x_w;
    this->b_q_w = w_q_b.normalized().conjugate();
    this->v_b = this->b_q_w * v_w;
    this->wBias = wBias;
    this->aBias = aBias;
    
    //DEBUG_PRINT("SetUpdate att_filter\n");

    return 0;
}

int AttFilter::
  SetUpdate(double tValid,
            const Vector3& x_w,
            const Quaternion& w_q_b,
            const Vector3& v_w,
            const Vector3& wBias,
            const Vector3& aBias,
            const Quaternion& b_q_g,
            const Matrix3& aGyrInv,
            const Matrix3& aAccInv) {
    int stat = SetUpdate(tValid, x_w, w_q_b, v_w, wBias, aBias);
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

int AttFilter::
  PropagateState() {
    /* NOTE(mereweth) - can only discard IMU data once state update with time
     * stamp after the IMU data stamp has been received
     */

    /* TODO(mereweth) - check for gaps in IMU data
     */

    // start at head of ring buffer, go to tail
    // for each sample:
        // peek at sample
        // if older than tLastUpdate, discard
        // optionally:
            // add sample to filter
            // pull latest filter element
        // optionally:
            // bump detection (based on vehicle model)
        // optionally:
            // apply rotation matrix, scale, and non-orthogonality corrections
        // offset by bias
        // TODO(mereweth) - check dt with exponential filter

    //DEBUG_PRINT("PropagateState att_filter\n");
    
    // discard samples from before last update
    while (this->imuBuf.size() &&
           (this->tLastUpdate > this->imuBuf.getFirstIn()->t)) {
        this->imuBuf.remove();
    }

    while (this->magBuf.size() &&
           (this->tLastUpdate > this->magBuf.getFirstIn()->t)) {
        this->magBuf.remove();
    }

    //TODO(mereweth) - restructure to get earliest of mag & IMU

    // NOTE(mereweth) - need to keep all IMU samples since last update
    // in case another update from right after it comes in
    for (int i = 0; i < this->imuBuf.size(); i++) {
        const quest_gnc::ImuSample* imu = this->imuBuf.get(i);
        FW_ASSERT(imu != NULL);

        // TODO(mereweth) - parameter for small time delta
        if (imu->t < this->tLastIntegrated + 1e-6) {
            continue;
        }
        
        this->tLastIntegrated = imu->t;

        const Vector3 a_b__norm = imu->a_b.normalized();

        // set initial orientation estimate from acceleration vector
        if (!this->initialized) {
            if (a_b__norm(2) >= 0.0) {
                this->b_q_w.w() = sqrt((a_b__norm(2) + 1) * 0.5);
                this->b_q_w.x() = -a_b__norm(1) / (2.0 * this->b_q_w.w());
                this->b_q_w.y() = a_b__norm(0) / (2.0 * this->b_q_w.w());
                this->b_q_w.z() = 0.0;
            }
            else {
                const FloatingPoint x = sqrt((1 - a_b__norm(2)) * 0.5);
                this->b_q_w.w() = -a_b__norm(1) / (2.0 * x);
                this->b_q_w.x() = x;
                this->b_q_w.y() = 0.0;
                this->b_q_w.z() = a_b__norm(0) / (2.0 * x);
            }
            this->omega_b__prev = imu->omega_b;
            this->initialized = true;
            return 0;
        }

        // check if in static state
        if ((fabs(imu->a_b.norm() - this->wParams.gravityMag) <= this->accelThresh) &&

            (fabs(imu->omega_b(0) - this->omega_b__prev(0)) <= this->omega_b__deltaThresh) &&
            (fabs(imu->omega_b(1) - this->omega_b__prev(1)) <= this->omega_b__deltaThresh) &&
            (fabs(imu->omega_b(2) - this->omega_b__prev(2)) <= this->omega_b__deltaThresh) &&

            (fabs(imu->omega_b(0) - this->wBias(0)) <= this->omega_b__thresh) &&
            (fabs(imu->omega_b(1) - this->wBias(1)) <= this->omega_b__thresh) &&
            (fabs(imu->omega_b(2) - this->wBias(2)) <= this->omega_b__thresh)) {
            this->wBias += this->biasAlpha * (imu->omega_b - this->wBias);
        }

        this->omega_b__prev = imu->omega_b;

        const Vector3 omega_b__unbias = imu->omega_b - this->wBias;
	// store unbiased angular velocity for odometry
	this->omega_b = omega_b__unbias;
        Quaternion omega_b__pureQuat;
        omega_b__pureQuat.w() = 0.0;
        omega_b__pureQuat.x() = omega_b__unbias(0);
        omega_b__pureQuat.y() = omega_b__unbias(1);
        omega_b__pureQuat.z() = omega_b__unbias(2);
        Quaternion b_q_w__pred = omega_b__pureQuat * this->b_q_w;
        b_q_w__pred.coeffs() *= -0.5 * this->dt;
        b_q_w__pred.coeffs() += this->b_q_w.coeffs();
        b_q_w__pred.normalize();

        const Vector3 a_g__pred = this->b_q_w.conjugate() * imu->a_b;
        Quaternion b_q_w__corr;
	
	if (a_g__pred(2) >= 0.0) {
	    b_q_w__corr.w() = sqrt((a_g__pred(2) + 1) * 0.5);
	    b_q_w__corr.x() = -a_g__pred(1) / (2.0 * b_q_w__corr.w());
	    b_q_w__corr.y() = a_g__pred(0) / (2.0 * b_q_w__corr.w());
	    b_q_w__corr.z() = 0.0;
	}
	else {
	    const FloatingPoint x = sqrt((1 - a_g__pred(2)) * 0.5);
	    b_q_w__corr.w() = -a_g__pred(1) / (2.0 * x);
	    b_q_w__corr.x() = x;
	    b_q_w__corr.y() = 0.0;
	    b_q_w__corr.z() = a_g__pred(0) / (2.0 * x);
	}
        b_q_w__corr.normalize();

        FloatingPoint factor;
        {
            const FloatingPoint a_mag = imu->a_b.squaredNorm();
            const FloatingPoint error = fabs(a_mag - this->wParams.gravityMag) / this->wParams.gravityMag;
            const FloatingPoint errStaticThresh = 0.1;
            const FloatingPoint errDynamicThresh = 0.2;
            const FloatingPoint m = 1.0 / (errStaticThresh - errDynamicThresh);
            const FloatingPoint b = 1.0 - m * errStaticThresh;
            if (error < errStaticThresh) {
                factor = 1.0;
            }
            else if (error < errDynamicThresh) {
                factor = m * error + b;
            }
            else {
                factor = 0.0;
            }
        }

        // Slerp (Spherical linear interpolation):
        {
	    if ((b_q_w__corr.w() > 1.0) ||
		(b_q_w__corr.w() < -1.0)) {
	        DEBUG_PRINT("fabs(b_q_w__corr.w()) > 1.0 in att_filter slerp");
  	        this->Reinit();
		return -1;
	    }
            const FloatingPoint angle = acos(b_q_w__corr.w());
	    const FloatingPoint sinAngle = fabs(sin(angle));
	    if (sinAngle < 1e-9) {
	        DEBUG_PRINT("fabs(sin(angle)) small in att_filter slerp");
  	        this->Reinit();
		return -1;
	    }
            const FloatingPoint gain = factor * this->accelGain;
            const FloatingPoint A = sin(angle * (1.0 - gain)) / sinAngle;
            const FloatingPoint B = sin(angle * gain) / sinAngle;
            b_q_w__corr.w() = A + B * b_q_w__corr.w();
            b_q_w__corr.x() = B * b_q_w__corr.x();
            b_q_w__corr.y() = B * b_q_w__corr.y();
            b_q_w__corr.z() = B * b_q_w__corr.z();
            b_q_w__corr.normalize();
        }

        // -------------------------- Position kinematics --------------------------

        const Vector3 a_b__temp = imu->a_b - this->aBias - this->b_q_w
                                  * Vector3(0, 0, this->wParams.gravityMag);

        this->v_b += a_b__temp * this->dt;
        this->x_w += this->b_q_w.conjugate() * this->v_b * this->dt;

        // NOTE(mereweth) - store final result of attitude filter step after position kinematics
        this->b_q_w = b_q_w__pred * b_q_w__corr;
        this->b_q_w.normalize();

    } // close for loop over buffered samples

    return 0;
}

} // namespace estimation NOLINT()
} // namespace quest_gnc NOLINT()
