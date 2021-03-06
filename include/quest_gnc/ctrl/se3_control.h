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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_SE3_CONTROL_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_SE3_CONTROL_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/se3_model.h"
#include "quest_gnc/utils/world_params.h"

#include "quest_gnc/utils/common.h"

namespace quest_gnc {

class Se3Control {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Se3Control();

    Se3Control(const Vector3& k_x,
               const Vector3& k_v,
               const Vector3& k_R,
               const Vector3& k_omega,
               Se3Model se3Model,
               WorldParams wParams);

    ~Se3Control();

  // ----------------------------------------------------------------------
  // Parameter, model, and gain setters
  // ----------------------------------------------------------------------

    int SetWorldParams(WorldParams wParams);

    int SetModel(Se3Model se3Model);

    int SetGains(const Vector3& k_x,
                 const Vector3& k_v,
                 const Vector3& k_R,
                 const Vector3& k_omega);

    int SetSaturation(const Vector3& sat_x,
                      const Vector3& sat_v,
                      const Vector3& sat_R,
                      const Vector3& sat_omega);

  // ----------------------------------------------------------------------
  // Thrust and moment getters
  // ----------------------------------------------------------------------

    int GetAccelAngAccelCommand(Vector3* a_w__comm,
                                Vector3* alpha_b__comm);

    int GetAccelCommand(Vector3* a_w__comm, bool velOnly = false);

    int GetAngAccelCommand(Vector3* alpha_b__comm,
                           bool rpVelOnly = false,
                           bool yawVelOnly = false);

    int GetAngAxisAlignedCommand(Vector3* alpha_b__comm,
			  	 unsigned char mask);

  // ----------------------------------------------------------------------
  // Feedback setters
  // ----------------------------------------------------------------------

    int SetOdometry(const Vector3& x_w,
                    const Quaternion& w_q_b,
                    const Vector3& v_b,
                    const Vector3& omega_b);

    int SetAttitudeAngVel(const Quaternion& w_q_b,
                          const Vector3& omega_b);

    int SetPositionLinVel(const Vector3& x_w,
                          const Vector3& v_b);

  // ----------------------------------------------------------------------
  // Command setters
  // ----------------------------------------------------------------------

    int SetPositionDes(const Vector3& x_w__des,
                       const Vector3& v_w__des,
                       const Vector3& a_w__des);

    int SetVelocityDes(const Vector3& v_w__des,
                       const Vector3& a_w__des);

    int SetAttitudeDes(const Quaternion& w_q_b__des,
                       const Vector3& omega_b__des,
		       const Vector3& alpha_b__des,
                       bool rpVelOnly = false,
                       bool yawVelOnly = false,
		       bool doSaturation = true);
    
    int SetAngVelDes(const Vector3& omega_b__des,
		     const Vector3& alpha_b__des,
		     bool doSaturation = true);

 private:
    // TODO(mereweth) - factor out into utils?
    // NOTE(mereweth) - uses, but does not modify, member variable
    void so3Error(Vector3* e_R, Vector3* e_omega,
                  bool rpVelOnly = false, bool yawVelOnly = false);
    
    int saturateAngular(bool rpVelOnly = false,
                        bool yawVelOnly = false); // NOTE(mereweth) - updates member variable
    
    int saturateLinear(); // NOTE(mereweth) - updates member variable

    // Parameters
    Vector3 k_x;
    Vector3 k_v;

    Vector3 k_R;
    Vector3 k_omega;

    Vector3 sat_x;
    Vector3 sat_v;
    Vector3 sat_R;
    Vector3 sat_omega;

    Se3Model se3Model;

    FloatingPoint invMass;
    Matrix3 inertia;

    WorldParams wParams;

    // Odometry

    Vector3 x_w;

    Matrix3 w_R_b;

    FloatingPoint yaw;

    Vector3 v_b;

    Vector3 omega_b;

    // Commanded

    Vector3 x_w__des;

    Vector3 v_w__des;

    Vector3 a_w__des;

    Matrix3 w_R_b__des;

    Vector3 omega_b__des;

    Vector3 alpha_b__des;

    // parameters object - warning tolerances, physical parameters
}; // class Se3Control NOLINT()

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_CTRL_SE3_CONTROL_H_
