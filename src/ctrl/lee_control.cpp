#include "../../include/quest_gnc/ctrl/lee_control.h"

#include "Eigen/Geometry"

namespace quest_gnc {
namespace multirotor {

LeeControl::LeeControl() {
    // TODO
}

LeeControl::~LeeControl() {
    // TODO
}

int LeeControl::
  SetOdometry(const Eigen::Vector3d& x_w,
              const Eigen::Quaterniond& w_q_b,
              const Eigen::Vector3d& v_b,
              const Eigen::Vector3d& omega_b) {
    this->x_w = x_w;
    this->w_q_b = w_q_b;
    this->v_b = v_b;
    this->omega_b = omega_b;
    return 0; // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetPositionDes(const Eigen::Vector3d& x_w__des,
                 const Eigen::Vector3d& v_w__des,
                 const Eigen::Vector3d& a_w__des) {
    this->x_w__des = x_w__des;
    this->v_w__des = v_w__des;
    this->a_w__des = a_w__des;
    return 0; // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetPositionAngAccelDes(const Eigen::Vector3d& x_w__des,
                         const Eigen::Vector3d& v_w__des,
                         const Eigen::Vector3d& alpha_b__des) {
    this->x_w__des = x_w__des;
    this->v_w__des = v_w__des;
    this->alpha_b__des = alpha_b__des;
    return 0; // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetVelocityDes(const Eigen::Vector3d& v_w__des,
                 const Eigen::Vector3d& a_w__des) {
    this->v_w__des = v_w__des;
    this->a_w__des = a_w__des;
    return 0; // TODO(mereweth) - sanitize inputs; return code
}

int LeeControl::
  SetAttitudeDes(const Eigen::Quaterniond& w_q_b__des,
                 const Eigen::Vector3d& omega_b__des) {
    this->w_q_b__des = w_q_b__des;
    this->omega_b__des = omega_b__des;
    return 0; // TODO(mereweth) - sanitize inputs; return code
}


int LeeControl::
  SetAttitudeAngAccelDes(const Eigen::Quaterniond& w_q_b__des,
                         const Eigen::Vector3d& omega_b__des,
                         const Eigen::Vector3d& alpha_b__des) {
    this->w_q_b__des = w_q_b__des;
    this->omega_b__des = omega_b__des;
    this->alpha_b__des = alpha_b__des;
    return 0; // TODO(mereweth) - sanitize inputs; return code
}

} // namespace multirotor
} // namespace quest_gnc
