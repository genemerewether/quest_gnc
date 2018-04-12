#include "quest_gnc/ctrl/lee_control.h"

#include "Eigen/Geometry"

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

namespace quest_gnc {
namespace multirotor {

LeeControl::LeeControl() :
    k_x(0, 0, 0), k_v(0, 0, 0),
    k_R(0, 0, 0), k_omega(0, 0, 0),
    mrModel(),
    invMass(1.0f),
    inertia(Eigen::Matrix3d::Identity()),
    wParams(),
    x_w(0, 0, 0), w_R_b(Eigen::Matrix3d::Identity()),
    v_b(0, 0, 0), omega_b(0, 0, 0),
    x_w__des(0, 0, 0), v_w__des(0, 0, 0), a_w__des(0, 0, 0),
    w_R_b__des(Eigen::Matrix3d::Identity()),
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
  SetModel(MultirotorModel mrModel) {
    // TODO(mereweth) - store smallnum as parameter
    // TODO(mereweth) - use return code here instead of assert?
    FW_ASSERT(mrModel.mass > 1e-6);
    this->mrModel = mrModel;

    this->invMass = static_cast<double>(1.0f) / this->mrModel.mass;

    return 0;
}

int LeeControl::
  SetGains(const Eigen::Vector3d& k_x,
           const Eigen::Vector3d& k_v,
           const Eigen::Vector3d& k_R,
           const Eigen::Vector3d& k_omega) {
    // TODO(mereweth) - check all positive

    this->k_x = k_x;
    this->k_v = k_v;
    this->k_R = k_R;
    this->k_omega = k_omega;

    return 0;
}

// ----------------------------------------------------------------------
// Thrust and moment getters
// ----------------------------------------------------------------------

int LeeControl::
  GetAccelAngAccelCommand(Eigen::Vector3d* a_w__comm,
                          Eigen::Vector3d* alpha_b__comm) {
    FW_ASSERT(a_w__comm);
    FW_ASSERT(alpha_b__comm);

    (void) GetAccelCommand(a_w__comm);

    (void) GetAngAccelCommand(alpha_b__comm);

    return 0;
}

int LeeControl::
  GetAccelCommand(Eigen::Vector3d* a_w__comm) {
    FW_ASSERT(a_w__comm);

    // TODO(mereweth) - flag for missing odometry or setpoint

    const Eigen::Vector3d x_w__err = this->x_w__des - this->x_w;

    // NOTE(mereweth) - sensitive to orientation estimate and map alignment
    const Eigen::Vector3d v_w__err = this->v_w__des - this->w_R_b * this->v_b;

    // TODO(mereweth) remove invMass here and make gains account for mass?
    *a_w__comm = (x_w__err.cwiseProduct(this->k_x)
                  + v_w__err.cwiseProduct(this->k_v)) * invMass;
                  + wParams.gravityMag * Eigen::Vector3d::UnitZ()
                  + this->a_w__des;

    // TODO(mereweth) - check return value
    (void) this->bodyFrame.FromYawAccel(yaw_des, *a_w__comm, &this->w_R_b__des);

    return 0;
}

int LeeControl::
  GetAngAccelCommand(Eigen::Vector3d* alpha_b__comm) {
    FW_ASSERT(alpha_b__comm);

    // TODO(mereweth) - check skew-symmetric, refactor vee operator as util
    const Eigen::Matrix3d e_R__hat = 0.5 * (this->w_R_b.transpose()
                                            * this->w_R_b__des
                                            - this->w_R_b__des.transpose()
                                            * this->w_R_b);
    const Eigen::Vector3d e_R(e_R__hat(2, 1), e_R__hat(0, 2), e_R__hat(1, 0));

    const Eigen::Vector3d e_omega = this->w_R_b.transpose() * this->w_R_b__des
                                    * this->omega_b__des - this->omega_b;

    // TODO(mereweth) - refactor hat operator as util
    Eigen::Matrix3d omega_b__hat;
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
  SetOdometry(const Eigen::Vector3d& x_w,
              const Eigen::Quaterniond& w_q_b,
              const Eigen::Vector3d& v_b,
              const Eigen::Vector3d& omega_b) {
    this->x_w = x_w;
    this->w_R_b = w_q_b.toRotationMatrix();
    this->v_b = v_b;
    this->omega_b = omega_b;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeAngVel(const Eigen::Quaterniond& w_q_b,
                    const Eigen::Vector3d& omega_b) {
    this->w_R_b = w_q_b.toRotationMatrix();
    this->omega_b = omega_b;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetPositionLinVel(const Eigen::Vector3d& x_w,
                    const Eigen::Vector3d& v_b) {
    this->x_w = x_w;
    this->v_b = v_b;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

// ----------------------------------------------------------------------
// Command setters
// ----------------------------------------------------------------------

int LeeControl::
  SetPositionDes(const Eigen::Vector3d& x_w__des,
                 const Eigen::Vector3d& v_w__des,
                 const Eigen::Vector3d& a_w__des) {
    this->x_w__des = x_w__des;
    this->v_w__des = v_w__des;
    this->a_w__des = a_w__des;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetYawDes(double yaw_des) {
    this->yaw_des = yaw_des;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetVelocityDes(const Eigen::Vector3d& v_w__des,
                 const Eigen::Vector3d& a_w__des) {
    this->v_w__des = v_w__des;
    this->a_w__des = a_w__des;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeDes(const Eigen::Quaterniond& w_q_b__des,
                 const Eigen::Vector3d& omega_b__des) {
    this->w_R_b__des = w_q_b__des.toRotationMatrix();
    this->omega_b__des = omega_b__des;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeAngAccelDes(const Eigen::Quaterniond& w_q_b__des,
                         const Eigen::Vector3d& omega_b__des,
                         const Eigen::Vector3d& alpha_b__des) {
    this->w_R_b__des = w_q_b__des.toRotationMatrix();
    this->omega_b__des = omega_b__des;
    this->alpha_b__des = alpha_b__des;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetPositionAngAccelDes(const Eigen::Vector3d& x_w__des,
                         const Eigen::Vector3d& v_w__des,
                         const Eigen::Vector3d& alpha_b__des) {
    this->x_w__des = x_w__des;
    this->v_w__des = v_w__des;
    this->alpha_b__des = alpha_b__des;
    return 0;

    // TODO(mereweth) - sanitize inputs; return code
}

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()
