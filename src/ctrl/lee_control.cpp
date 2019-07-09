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

#include "quest_gnc/utils/so3.h"

#include "Eigen/Geometry"

#include <math.h>

#include <stdio.h>
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
    v_b(0, 0, 0), a_b(0, 0, 0), omega_b(0, 0, 0),
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
    if (mrModel.rigidBody.mass < 1e-6) {
        return -1;
    }
    this->mrModel = mrModel;

    this->invMass = static_cast<FloatingPoint>(1.0f) / this->mrModel.rigidBody.mass;

    // TODO(mereweth) - check all positive
    
    this->inertia(0, 0) = this->mrModel.rigidBody.Ixx;
    this->inertia(1, 1) = this->mrModel.rigidBody.Iyy;
    this->inertia(2, 2) = this->mrModel.rigidBody.Izz;

    this->inertia(0, 1) = this->mrModel.rigidBody.Ixy;
    this->inertia(1, 0) = this->mrModel.rigidBody.Ixy;
    this->inertia(0, 2) = this->mrModel.rigidBody.Ixz;
    this->inertia(2, 0) = this->mrModel.rigidBody.Ixz;
    this->inertia(1, 2) = this->mrModel.rigidBody.Iyz;
    this->inertia(2, 1) = this->mrModel.rigidBody.Iyz;

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

    
    DEBUG_PRINT("sat_x [%f, %f, %f]\n",
                this->sat_x(0), this->sat_x(1), this->sat_x(2));
    DEBUG_PRINT("sat_v [%f, %f, %f]\n",
                this->sat_v(0), this->sat_v(1), this->sat_v(2));
    DEBUG_PRINT("sat_R [%f, %f, %f]\n",
                this->sat_R(0), this->sat_R(1), this->sat_R(2));
    DEBUG_PRINT("sat_omega [%f, %f, %f]\n",
                this->sat_omega(0), this->sat_omega(1), this->sat_omega(2));
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

    if (velOnly) {
        *a_w__comm = v_w__err.cwiseProduct(this->k_v) * this->invMass;
    }
    else {
        *a_w__comm = (x_w__err.cwiseProduct(this->k_x)
                      + v_w__err.cwiseProduct(this->k_v)) * this->invMass;
    }
    *a_w__comm += wParams.gravityMag * Vector3::UnitZ() + this->a_w__des;

    // TODO(mereweth) - check return value
    (void) this->bodyFrame.FromYawAccel(yaw_des, *a_w__comm, &this->w_R_b__des);

    // NOTE(mereweth) - set angular velocity desired based on position error dynamics
    // NOTE(mereweth) - sensitive to orientation estimate and map alignment
    // TODO(mereweth) - test in simple sim with gaussian noise - is this term worth including?
    const Vector3 a_w__err(0, 0, 0);// = this->a_w__des - this->w_R_b * this->a_b;
    Vector3 a_w__comm_dot;
    if (velOnly) {
        a_w__comm_dot = a_w__err.cwiseProduct(this->k_v) * this->invMass;
    }
    else {
        a_w__comm_dot = (v_w__err.cwiseProduct(this->k_x)
                         + a_w__err.cwiseProduct(this->k_v)) * this->invMass;
    }
    // TODO(mereweth) - test in simple sim with tracking error - is feedforward appropriate?
    //a_w__comm_dot += this->j_w__des;
    // -y_body dot prod z_body_dot
    this->omega_b__des(0) = -this->w_R_b__des.col(1).dot(a_w__comm_dot) / a_w__comm->norm();
    // x_body dot prod z_body_dot
    this->omega_b__des(1) = this->w_R_b__des.col(0).dot(a_w__comm_dot) / a_w__comm->norm();
    // TODO(mereweth) - calculate z body des
    this->omega_b__des(2) = 0.0;
    
    // NOTE(mereweth) - commanded attitude is set above, so give error code if saturated
    return this->saturateAngular();
}

int LeeControl::
  GetAngAccelCommand(Vector3* alpha_b__comm,
                     bool rpVelOnly,
                     bool yawVelOnly) {
    FW_ASSERT(alpha_b__comm);

    Vector3 e_R;
    Vector3 e_omega;
    this->so3Error(&e_R, &e_omega,
                   rpVelOnly, yawVelOnly);

    *alpha_b__comm = e_R.cwiseProduct(this->k_R)
                     + e_omega.cwiseProduct(this->k_omega)
                     + this->omega_b.cross(this->inertia * this->omega_b);

    if (!rpVelOnly && !yawVelOnly) {
        Matrix3 omega_b__hat;
        hat3(this->omega_b, &omega_b__hat);
        *alpha_b__comm -= this->inertia * (omega_b__hat * this->w_R_b.transpose()
                                           * this->w_R_b__des * this->omega_b__des
                                           - this->w_R_b.transpose()
                                             * this->w_R_b__des
                                             * this->alpha_b__des);
    }
    // TODO(mereweth) - check that alpha_b__des == omega_b__des__dot

    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  GetAngAxisAlignedCommand(Vector3* alpha_b__comm,
			   unsigned char mask) {
    Vector3 e_omega = this->omega_b__des - this->omega_b;
    for (unsigned int i = 0; i < 3; i++) {
        (*alpha_b__comm)(i) = 0.0;
        if (mask & (1 << i)) {
            FloatingPoint angle = 0.0;
            getUnitAngle(&angle, this->w_R_b, i);
            FloatingPoint angle_des = 0.0;
            getUnitAngle(&angle_des, this->w_R_b__des, i);
            
            FloatingPoint angleDiff = angle_des - angle;
            wrapAngle(&angleDiff);
            (*alpha_b__comm)(i) += angleDiff * this->k_R(i);
            (*alpha_b__comm)(i) += e_omega(i) * this->k_omega(i);
        }
    }
    
    return 0;
}

int LeeControl::
  GetDesired(Vector3* x_w__des,
             Quaternion* w_q_b__des,
             Vector3* v_b__des,
             Vector3* omega_b__des) {
    FW_ASSERT(x_w__des);
    FW_ASSERT(w_q_b__des);
    FW_ASSERT(v_b__des);
    FW_ASSERT(omega_b__des);

    *x_w__des = this->x_w__des;
    *w_q_b__des = this->w_R_b__des;
    *v_b__des = this->w_R_b.inverse() * this->v_w__des;
    *omega_b__des = this->omega_b__des;
    return 0;
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

    getUnitAngle(&this->yaw, this->w_R_b, 2);
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeAngVel(const Quaternion& w_q_b,
                    const Vector3& omega_b) {
    this->w_R_b = w_q_b.toRotationMatrix();    
    this->omega_b = omega_b;

    getUnitAngle(&this->yaw, this->w_R_b, 2);
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetPositionLinVelAcc(const Vector3& x_w,
                       const Vector3& v_b,
                       const Vector3& a_b) {
    this->x_w = x_w;
    this->v_b = v_b;
    this->a_b = a_b;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

// ----------------------------------------------------------------------
// Command setters
// ----------------------------------------------------------------------

int LeeControl::
  SetPositionDes(const Vector3& x_w__des,
                 const Vector3& v_w__des,
                 const Vector3& a_w__des,
                 const Vector3& j_w__des) {
    this->x_w__des = x_w__des;
    this->v_w__des = v_w__des;
    this->a_w__des = a_w__des;
    this->j_w__des = j_w__des;
    
    return this->saturateLinear();

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetYawDes(FloatingPoint yaw_des) {
    wrapAngle(&yaw_des);  
    FloatingPoint yawDiff = yaw_des - this->yaw;
    wrapAngle(&yawDiff);

    if (fabs(yawDiff) > this->sat_yaw) {
        DEBUG_PRINT("yawDiff %f, yaw_des %f, this->yaw %f\n",
                    yawDiff, yaw_des, this->yaw);
        this->yaw_des = this->yaw + ((yawDiff > 0.0) ? this->sat_yaw : -this->sat_yaw);
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
                 const Vector3& a_w__des,
                 const Vector3& j_w__des) {
    this->v_w__des = v_w__des;
    this->a_w__des = a_w__des;
    this->j_w__des = j_w__des;
    
    return this->saturateLinear();

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeDes(const Quaternion& w_q_b__des,
                 const Vector3& omega_b__des,
                 bool rpVelOnly,
                 bool yawVelOnly,
		 bool doSaturation) {  
    this->w_R_b__des = w_q_b__des.toRotationMatrix();
    this->omega_b__des = omega_b__des;

    if (doSaturation) {
        return this->saturateAngular(rpVelOnly, yawVelOnly);
    }
    else {
        return 0;
    }

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
  
void LeeControl::
  so3Error(Vector3* e_R, Vector3* e_omega,
           bool rpVelOnly,
           bool yawVelOnly) {
    FW_ASSERT(e_R);
    FW_ASSERT(e_omega);
    const Matrix3 e_R__hat = 0.5 * (this->w_R_b.transpose()
                                            * this->w_R_b__des
                                            - this->w_R_b__des.transpose()
                                            * this->w_R_b);
    // TODO(mereweth) - return error code
    vee3(e_R, e_R__hat);

    if (rpVelOnly || yawVelOnly) {
        *e_omega = this->omega_b__des - this->omega_b;
    }
    else {
        *e_omega = this->w_R_b.transpose() * this->w_R_b__des
                   * this->omega_b__des - this->omega_b;
    }
    
    if (rpVelOnly) {
        e_R->x() = 0;
        e_R->y() = 0;
    }
    
    if (yawVelOnly) {
        e_R->z() = 0;
    }
}

int LeeControl::
  saturateAngular(bool rpVelOnly,
                  bool yawVelOnly) {
    Vector3 e_R;
    Vector3 e_omega;
    this->so3Error(&e_R, &e_omega, rpVelOnly, yawVelOnly);
    
    DEBUG_PRINT("e_R [%f, %f, %f], sat_R [%f, %f, %f]\n",
                this->e_R(0), this->e_R(1), this->e_R(2),
                this->sat_R(0), this->sat_R(1), this->sat_R(2));
    const Vector3 e_R__clamped = (e_R.cwiseAbs().array() > this->sat_R.array()).select(
              (e_R.array() > 0.0).select(this->sat_R, -this->sat_R),
              e_R);
    DEBUG_PRINT("e_R__clamped [%f, %f, %f]\n",
                this->e_R__clamped(0), this->e_R__clamped(1), this->e_R__clamped(2));

    Matrix3 expMap;
    expMap3(e_R__clamped, &expMap);
    this->w_R_b__des = this->w_R_b * expMap;
    
    DEBUG_PRINT("omega [%f, %f, %f], sat_omega [%f, %f, %f]\n",
                this->omega_b(0), this->omega_b(1), this->omega_b(2),
                this->sat_omega(0), this->sat_omega(1), this->sat_omega(2));
    DEBUG_PRINT("omega_des [%f, %f, %f]\n",
                this->omega_b__des(0), this->omega_b__des(1), this->omega_b__des(2));
    this->omega_b__des = this->w_R_b__des.transpose() * this->w_R_b * (this->omega_b
      + (e_omega.cwiseAbs().array() > this->sat_omega.array()).select(
              (e_omega.array() > 0.0).select(this->sat_omega, -this->sat_omega),
              e_omega));
    DEBUG_PRINT("omega_des after [%f, %f, %f]\n",
                this->omega_b__des(0), this->omega_b__des(1), this->omega_b__des(2));

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

    DEBUG_PRINT("x_w [%f, %f, %f], sat_x [%f, %f, %f]\n",
                this->x_w(0), this->x_w(1), this->x_w(2),
                this->sat_x(0), this->sat_x(1), this->sat_x(2));
    DEBUG_PRINT("x_w_des [%f, %f, %f]\n",
                this->x_w__des(0), this->x_w__des(1), this->x_w__des(2));
    this->x_w__des = this->x_w
      + (e_x.cwiseAbs().array() > this->sat_x.array()).select(
              (e_x.array() > 0.0).select(this->sat_x, -this->sat_x),
              e_x);
    DEBUG_PRINT("x_w__des after [%f, %f, %f]\n",
                this->x_w__des(0), this->x_w__des(1), this->x_w__des(2));

    DEBUG_PRINT("v_w [%f, %f, %f], sat_v [%f, %f, %f]\n",
                v_w(0), v_w(1), v_w(2),
                this->sat_v(0), this->sat_v(1), this->sat_v(2));
    DEBUG_PRINT("v_w_des [%f, %f, %f]\n",
                this->v_w__des(0), this->v_w__des(1), this->v_w__des(2));
    this->v_w__des = v_w
      + (e_v.cwiseAbs().array() > this->sat_v.array()).select(
              (e_v.array() > 0.0).select(this->sat_v, -this->sat_v),
              e_v);
    DEBUG_PRINT("v_w__des after [%f, %f, %f]\n",
                this->v_w__des(0), this->v_w__des(1), this->v_w__des(2));
    
    if ((e_x.cwiseAbs().array() > this->sat_x.array()).any() ||
        (e_v.cwiseAbs().array() > this->sat_v.array()).any()) {
        return -1;
    }
    return 0;
}
  
} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()
