#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_LEE_CONTROL_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_LEE_CONTROL_H_

#include <Eigen/Eigen>

#include "quest_gnc/diffeo/rates.h"
#include "quest_gnc/diffeo/body_frame.h"
#include "quest_gnc/utils/multirotor_model.h"
#include "quest_gnc/utils/world_params.h"

namespace quest_gnc {
namespace multirotor {

class LeeControl {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LeeControl();

    LeeControl(const Eigen::Vector3d& k_x,
               const Eigen::Vector3d& k_v,
               const Eigen::Vector3d& k_R,
               const Eigen::Vector3d& k_omega,
               MultirotorModel mrModel,
               WorldParams wParams);

    ~LeeControl();

  // ----------------------------------------------------------------------
  // Thrust and moment getters
  // ----------------------------------------------------------------------

    // Valid in position and velocity mode
    int GetAccelAngAccelCommand(Eigen::Vector3d* a_w__comm,
                                Eigen::Vector3d* alpha_b__comm);

    // Valid in attitude mode
    int GetAngAccelCommand(Eigen::Vector3d* alpha_b__comm);

  // ----------------------------------------------------------------------
  // Feedback setters
  // ----------------------------------------------------------------------

    int SetOdometry(const Eigen::Vector3d& x_w,
                    const Eigen::Quaterniond& w_q_b,
                    const Eigen::Vector3d& v_b,
                    const Eigen::Vector3d& omega_b);

  // ----------------------------------------------------------------------
  // Command setters
  // ----------------------------------------------------------------------

    int SetPositionDes(const Eigen::Vector3d& x_w__des,
                       const Eigen::Vector3d& v_w__des,
                       const Eigen::Vector3d& a_w__des);

    int SetVelocityDes(const Eigen::Vector3d& v_w__des,
                       const Eigen::Vector3d& a_w__des);

    int SetYawDes(double yaw_des);

    int SetAttitudeDes(const Eigen::Quaterniond& w_q_b__des,
                       const Eigen::Vector3d& omega_b__des);

    int SetAttitudeAngAccelDes(const Eigen::Quaterniond& w_q_b__des,
                               const Eigen::Vector3d& omega_b__des,
                               const Eigen::Vector3d& alpha_b__des);

    int SetPositionAngAccelDes(const Eigen::Vector3d& x_w__des,
                               const Eigen::Vector3d& v_w__des,
                               const Eigen::Vector3d& alpha_b__des);

 private:
    // Parameters
    Eigen::Vector3d k_x;
    Eigen::Vector3d k_v;

    Eigen::Vector3d k_R;
    Eigen::Vector3d k_omega;

    MultirotorModel mrModel;

    double invMass;
    Eigen::Matrix3d inertia;

    WorldParams wParams;

    // Odometry

    Eigen::Vector3d x_w;

    Eigen::Matrix3d w_R_b;

    Eigen::Vector3d v_b;

    Eigen::Vector3d omega_b;

    // Commanded

    //! position mode only
    Eigen::Vector3d x_w__des;

    //! position/velocity modes only
    Eigen::Vector3d v_w__des;

    //! position/velocity modes only
    Eigen::Vector3d a_w__des;

    //! position/velocity modes only
    double yaw_des;

    //! explicitly set in attitude mode only
    Eigen::Matrix3d w_R_b__des;

    //! explicitly set in attitude mode only
    Eigen::Vector3d omega_b__des;

    //! explicitly set in angular acceleration feedforward mode only
    Eigen::Vector3d alpha_b__des;

    // Math implementations
    BodyFrame bodyFrame;

    Rates rates;

    // parameters object - warning tolerances, physical parameters
}; // class LeeControl NOLINT()

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_LEE_CONTROL_H_
