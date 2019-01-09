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

#include "quest_gnc/ctrl/lee_control.h"

#include "Eigen/Geometry"

#ifndef M_PI
#ifdef BUILD_DSPAL
#define M_PI 3.14159265358979323846
#endif
#endif

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

#define DEBUG_PRINT(x,...)
//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)

namespace quest_gnc {
namespace multirotor {

LeeControl::LeeControl() :
    k_x(0, 0, 0), k_v(0, 0, 0),
    k_R(0, 0, 0), k_omega(0, 0, 0),
    sat_x(0, 0, 0), sat_v(0, 0, 0),
    sat_R(0, 0, 0), sat_omega(0, 0, 0),
    sat_yaw(0.0),
    mrModel(),
    invMass(1.0f),
    inertia(Matrix3::Identity()),
    wParams(),
    x_w(0, 0, 0), w_R_b(Matrix3::Identity()),
    v_b(0, 0, 0), omega_b(0, 0, 0),
    x_w__des(0, 0, 0), v_w__des(0, 0, 0), a_w__des(0, 0, 0),
    w_R_b__des(Matrix3::Identity()),
    omega_b__des(0, 0, 0), alpha_b__des(0, 0, 0),
    bodyFrame(),
    rates() {
}

LeeControl::~LeeControl() {
    // TODO(mereweth)
}

// ----------------------------------------------------------------------
// Parameter, model, and gain setters
// ----------------------------------------------------------------------

int LeeControl::
  SetWorldParams(WorldParams wParams) {
    this->wParams = wParams;

    return 0;
}

int LeeControl::
  SetModel(MultirotorModel mrModel) {
    // TODO(mereweth) - store smallnum as parameter
    if (mrModel.mass < 1e-6) {
        return -1;
    }
    this->mrModel = mrModel;

    this->invMass = static_cast<FloatingPoint>(1.0f) / this->mrModel.mass;

    // TODO(mereweth) - check all positive
    
    this->inertia(0, 0) = this->mrModel.Ixx;
    this->inertia(1, 1) = this->mrModel.Iyy;
    this->inertia(2, 2) = this->mrModel.Izz;

    this->inertia(0, 1) = this->mrModel.Ixy;
    this->inertia(1, 0) = this->mrModel.Ixy;
    this->inertia(0, 2) = this->mrModel.Ixz;
    this->inertia(2, 0) = this->mrModel.Ixz;
    this->inertia(1, 2) = this->mrModel.Iyz;
    this->inertia(2, 1) = this->mrModel.Iyz;

    return 0;
}

int LeeControl::
  SetGains(const Vector3& k_x,
           const Vector3& k_v,
           const Vector3& k_R,
           const Vector3& k_omega) {
    this->k_x = k_x.cwiseAbs();
    this->k_v = k_v.cwiseAbs();
    this->k_R = k_R.cwiseAbs();
    this->k_omega = k_omega.cwiseAbs();

    return 0;
}

int LeeControl::
  SetSaturation(const Vector3& sat_x,
                const Vector3& sat_v,
                const Vector3& sat_R,
                const Vector3& sat_omega,
                FloatingPoint sat_yaw) {
    this->sat_x = sat_x.cwiseAbs();
    this->sat_v = sat_v.cwiseAbs();
    this->sat_R = sat_R.cwiseAbs();
    this->sat_omega = sat_omega.cwiseAbs();
    this->sat_yaw = fabs(sat_yaw);

    return 0;
}

// ----------------------------------------------------------------------
// Thrust and moment getters
// ----------------------------------------------------------------------

int LeeControl::
  GetAccelAngAccelCommand(Vector3* a_w__comm,
                          Vector3* alpha_b__comm) {
    FW_ASSERT(a_w__comm);
    FW_ASSERT(alpha_b__comm);

    int stat = GetAccelCommand(a_w__comm);

    (void) GetAngAccelCommand(alpha_b__comm);

    return stat;
}

int LeeControl::
  GetAccelCommand(Vector3* a_w__comm, bool velOnly) {
    FW_ASSERT(a_w__comm);

    // TODO(mereweth) - flag for missing odometry or setpoint

    const Vector3 x_w__err = this->x_w__des - this->x_w;

    // NOTE(mereweth) - sensitive to orientation estimate and map alignment
    const Vector3 v_w__err = this->v_w__des - this->w_R_b * this->v_b;

    // TODO(mereweth) remove invMass here and make gains account for mass?
    if (velOnly) {
        *a_w__comm = v_w__err.cwiseProduct(this->k_v) * invMass;
    }
    else {
        *a_w__comm = (x_w__err.cwiseProduct(this->k_x)
                      + v_w__err.cwiseProduct(this->k_v)) * invMass;
    }
    *a_w__comm += wParams.gravityMag * Vector3::UnitZ() + this->a_w__des;

    // TODO(mereweth) - check return value
    (void) this->bodyFrame.FromYawAccel(yaw_des, *a_w__comm, &this->w_R_b__des);

    // NOTE(mereweth) - commanded attitude is set above, so give error code if saturated
    return this->saturateAngular();
}

int LeeControl::
  GetAngAccelCommand(Vector3* alpha_b__comm) {
    FW_ASSERT(alpha_b__comm);

    Vector3 e_R;
    Vector3 e_omega;
    this->so3Error(&e_R, &e_omega);

    // TODO(mereweth) - refactor hat operator as util
    Matrix3 omega_b__hat;
    omega_b__hat << 0, -this->omega_b.z(), this->omega_b.y(),
                    this->omega_b.z(), 0, -this->omega_b.x(),
                    -this->omega_b.y(), this->omega_b.x(), 0;
    *alpha_b__comm = e_R.cwiseProduct(this->k_R)
                     + e_omega.cwiseProduct(this->k_omega)
                     + this->omega_b.cross(this->inertia * this->omega_b)
                     - this->inertia * (omega_b__hat * this->w_R_b.transpose()
                                        * this->w_R_b__des * this->omega_b__des
                                        - this->w_R_b.transpose()
                                          * this->w_R_b__des
                                          * this->alpha_b__des);
    // TODO(mereweth) - check that alpha_b__des == omega_b__des__dot

    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}
  
// ----------------------------------------------------------------------
// Feedback setters
// ----------------------------------------------------------------------

int LeeControl::
  SetOdometry(const Vector3& x_w,
              const Quaternion& w_q_b,
              const Vector3& v_b,
              const Vector3& omega_b) {
    this->x_w = x_w;
    this->w_R_b = w_q_b.toRotationMatrix();
    this->v_b = v_b;
    this->omega_b = omega_b;

    this->updateYaw();
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeAngVel(const Quaternion& w_q_b,
                    const Vector3& omega_b) {
    this->w_R_b = w_q_b.toRotationMatrix();    
    this->omega_b = omega_b;

    this->updateYaw();
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetPositionLinVel(const Vector3& x_w,
                    const Vector3& v_b) {
    this->x_w = x_w;
    this->v_b = v_b;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

// ----------------------------------------------------------------------
// Command setters
// ----------------------------------------------------------------------

int LeeControl::
  SetPositionDes(const Vector3& x_w__des,
                 const Vector3& v_w__des,
                 const Vector3& a_w__des) {
    this->x_w__des = x_w__des;
    this->v_w__des = v_w__des;
    this->a_w__des = a_w__des;
    
    return this->saturateLinear();

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetYawDes(FloatingPoint yaw_des) {

    if (yaw_des > M_PI) {
        yaw_des -= 2*M_PI;
    }
    else if (yaw_des < -M_PI) {
        yaw_des += 2*M_PI;
    }
  
    FloatingPoint yawDiff = yaw_des - this->yaw;
    if (yawDiff < -M_PI) {
        unsigned int num2pi = 1u + (unsigned int) fabs(yawDiff / (2 * M_PI));
        yawDiff += num2pi * 2 * M_PI;
    }
    else if (yawDiff > M_PI) {
        unsigned int num2pi = 1u + (unsigned int) fabs(yawDiff / (2 * M_PI));
        yawDiff -= num2pi * 2 * M_PI;
    }

    if (fabs(yawDiff) > this->sat_yaw) {
        DEBUG_PRINT("yawDiff %f, yaw_des %f, this->yaw %f\n",
                    yawDiff, yaw_des, this->yaw);
        this->yaw_des = this->yaw + (yawDiff > 0.0) ? this->sat_yaw : -this->sat_yaw;
        return -1;
    }
    else {
        this->yaw_des = yaw_des;
        return 0;
    }

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetVelocityDes(const Vector3& v_w__des,
                 const Vector3& a_w__des) {
    this->v_w__des = v_w__des;
    this->a_w__des = a_w__des;
    
    return this->saturateLinear();

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeDes(const Quaternion& w_q_b__des,
                 const Vector3& omega_b__des) {  
    this->w_R_b__des = w_q_b__des.toRotationMatrix();
    this->omega_b__des = omega_b__des;

    return this->saturateAngular();

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeAngAccelDes(const Quaternion& w_q_b__des,
                         const Vector3& omega_b__des,
                         const Vector3& alpha_b__des) {  
    this->w_R_b__des = w_q_b__des.toRotationMatrix();
    this->omega_b__des = omega_b__des;
    this->alpha_b__des = alpha_b__des;
    
    return this->saturateAngular();

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetPositionAngAccelDes(const Vector3& x_w__des,
                         const Vector3& v_w__des,
                         const Vector3& alpha_b__des) {
    this->x_w__des = x_w__des;
    this->v_w__des = v_w__des;
    this->alpha_b__des = alpha_b__des;
    
    return this->saturateLinear();

    // TODO(mereweth) - sanitize inputs; return code
}
  
// ----------------------------------------------------------------------
// Private helper functions
// ----------------------------------------------------------------------
  
// TODO(mereweth) - share in utils?
void LeeControl::
  updateYaw() {
    Vector3 xRot = Vector3::UnitX();
    xRot = this->w_R_b * xRot;
    Vector3 xRotProj = xRot - xRot.dot(Vector3::UnitZ()) * Vector3::UnitZ();
    xRotProj.normalize();
    this->yaw = acos(xRotProj.dot(Vector3::UnitX()));
    if (xRotProj(1) < 0.0) {
        this->yaw *= -1.0;
    }
}

void LeeControl::
  so3Error(Vector3* e_R, Vector3* e_omega) {
    FW_ASSERT(e_R);
    FW_ASSERT(e_omega);
    // TODO(mereweth) - check skew-symmetric, refactor vee operator as util
    const Matrix3 e_R__hat = 0.5 * (this->w_R_b.transpose()
                                            * this->w_R_b__des
                                            - this->w_R_b__des.transpose()
                                            * this->w_R_b);
    *e_R = Vector3(e_R__hat(2, 1), e_R__hat(0, 2), e_R__hat(1, 0));

    *e_omega = this->w_R_b.transpose() * this->w_R_b__des
               * this->omega_b__des - this->omega_b;
}

int LeeControl::
  saturateAngular() {
    Vector3 e_R;
    Vector3 e_omega;
    this->so3Error(&e_R, &e_omega);
    
    this->omega_b__des = this->omega_b
      + (e_omega.cwiseAbs().array() > this->sat_omega.array()).select(
              (e_omega.array() > 0.0).select(this->sat_omega, -this->sat_omega),
              e_omega);

    if ((e_omega.cwiseAbs().array() > this->sat_omega.array()).any() ||
        (e_R.cwiseAbs().array() > this->sat_R.array()).any()) {
        return -1;
    }
    return 0;
}
    
int LeeControl::
  saturateLinear() {
    Vector3 e_x = this->x_w__des - this->x_w;
    
    // NOTE(mereweth) - sensitive to orientation estimate and map alignment
    const Vector3 v_w = this->w_R_b * this->v_b;
    Vector3 e_v = this->v_w__des - v_w;
    
    this->x_w__des = this->x_w
      + (e_x.cwiseAbs().array() > this->sat_x.array()).select(
              (e_x.array() > 0.0).select(this->sat_x, -this->sat_x),
              e_x);
    
    this->v_w__des = v_w
      + (e_v.cwiseAbs().array() > this->sat_v.array()).select(
              (e_v.array() > 0.0).select(this->sat_v, -this->sat_v),
              e_v);
    
    if ((e_x.cwiseAbs().array() > this->sat_x.array()).any() ||
        (e_v.cwiseAbs().array() > this->sat_v.array()).any()) {
        return -1;
    }
    return 0;
}
  
} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()
