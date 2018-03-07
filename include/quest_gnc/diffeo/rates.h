#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_RATES_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_RATES_H_

#include <Eigen/Eigen>

namespace quest_gnc {
namespace multirotor {

class Rates {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Rates();

    ~Rates();

    int GetPDotQDot(double thrust,
                    const Eigen::Vector3d& jerk_w,
                    const Eigen::Vector3d& snap_w,
                    const Eigen::Matrix3d& w_R_b,
                    const Eigen::Vector3d& omega_b,
                    double* pDot,
                    double* qDot) const;

    int GetPQ(double thrust,
              const Eigen::Vector3d& jerk_w,
              const Eigen::Matrix3d& w_R_b,
              double* p,
              double* q) const;

    int ProjectYawDerivToBody(double yaw_deriv,
                              const Eigen::Vector3d& zBody_w,
                              double* body_z_deriv) const;

 private:
    // parameters object - warning tolerances, physical parameters

}; // class Rates

} // namespace multirotor
} // namespace quest_gnc

#endif // QUEST_GNC_INCLUDE_QUEST_GNC_DIFFEO_RATES_H_
