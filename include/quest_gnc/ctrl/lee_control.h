#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_LEE_CONTROL_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_LEE_CONTROL_H_

#include <Eigen/Eigen>

namespace quest_gnc {
namespace multirotor {

class LeeControl {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LeeControl();

    ~LeeControl();

    int SetOdometry(const Eigen::Vector3d& x_w,
                    const Eigen::Quaterniond& w_q_b,
                    const Eigen::Vector3d& v_b,
                    const Eigen::Vector3d& omega_b);

    int SetPositionDes(const Eigen::Vector3d& x_w__des,
                       const Eigen::Vector3d& v_w__des,
                       const Eigen::Vector3d& a_w__des);

    int SetPositionAngAccelDes(const Eigen::Vector3d& x_w__des,
                               const Eigen::Vector3d& v_w__des,
                               const Eigen::Vector3d& alpha_b__des);

    int SetVelocityDes(const Eigen::Vector3d& v_w__des,
                       const Eigen::Vector3d& a_w__des);

    int SetAttitudeDes(const Eigen::Quaterniond& w_q_b__des,
                       const Eigen::Vector3d& omega_b__des);

    int SetAttitudeAngAccelDes(const Eigen::Quaterniond& w_q_b__des,
                               const Eigen::Vector3d& omega_b__des,
                               const Eigen::Vector3d& alpha_b__des);

  private:

    // Odometry

    Eigen::Vector3d x_w;

    Eigen::Quaterniond w_q_b;

    Eigen::Vector3d v_b;

    Eigen::Vector3d omega_b;

    // Commanded

    //! position mode only
    Eigen::Vector3d x_w__des;

    //! position/velocity modes only
    Eigen::Vector3d v_w__des;

    //! position/velocity modes only
    Eigen::Vector3d a_w__des;

    //! attitude mode only
    Eigen::Quaterniond w_q_b__des;

    //! attitude mode only
    Eigen::Vector3d omega_b__des;

    //! angular acceleration feedforward mode only
    Eigen::Vector3d alpha_b__des;

    // parameters object - warning tolerances, physical parameters

}; // class LeeControl

} // namespace multirotor
} // namespace quest_gnc

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_LEE_CONTROL_H_
