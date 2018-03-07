#ifndef QUEST_GNC_DIFFEO_RATES_H
#define QUEST_GNC_DIFFEO_RATES_H

#include <Eigen/Eigen>

namespace quest_gnc {
namespace multirotor {

class Rates {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Rates();

    ~Rates();

    int GetPDotQDot(double thrust,
                    const Eigen::Vector3d& jerk,
                    const Eigen::Vector3d& snap,
                    const Eigen::Matrix3d& w_R_body,
                    const Eigen::Vector3d& omega,
                    double& pDot,
                    double& qDot) const;

    int GetPQ(double thrust,
              const Eigen::Vector3d& jerk,
              const Eigen::Matrix3d& w_R_body,
              double& p,
              double& q) const;

    int ProjectYawDerivToBody(double yaw_deriv,
                              const Eigen::Vector3d& zBody_w,
                              double& body_z_deriv) const;

  private:
    // parameters object
}; // class Rates

} // namespace multirotor
} // namespace quest_gnc

#endif // #ifndef QUEST_GNC_DIFFEO_RATES_H
