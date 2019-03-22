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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_MIXER_BASIC_MIXER_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_MIXER_BASIC_MIXER_H_

#include <Eigen/Eigen>

//#include "quest_gnc/diffeo/rates.h"
//#include "quest_gnc/diffeo/body_frame.h"
//#include "quest_gnc/utils/multirotor_model.h"
//#include "quest_gnc/utils/world_params.h"

#include "quest_gnc/utils/common.h"
#include "quest_gnc/mixer/basic_mixer_cfg.h"

namespace quest_gnc {
namespace multirotor {

typedef Eigen::Matrix<FloatingPoint, kBasicMixerMaxActuators, 4> PinvMixMatrix;
typedef Eigen::Matrix<FloatingPoint, 4, kBasicMixerMaxActuators> MixMatrix;
typedef Eigen::Matrix<FloatingPoint, kBasicMixerMaxActuators, 1> MixOutput;
  
class BasicMixer {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BasicMixer();

    BasicMixer(MixMatrix mixer,
	       unsigned int numActuators);

    ~BasicMixer();

  // ----------------------------------------------------------------------
  // Parameter, model, and gain setters
  // ----------------------------------------------------------------------

  // Will calculate pinv of mixer
    int SetMixer(MixMatrix mixer,
		 unsigned int numActuators);
    
  // ----------------------------------------------------------------------
  // Rotational velocity command
  // ----------------------------------------------------------------------

    int GetRotorVelCommand(MixOutput* rotVel__comm);

  // ----------------------------------------------------------------------
  // Torque/moment setter
  // ----------------------------------------------------------------------

    int SetTorqueThrustDes(const Vector3& moment_b__des,
                           const Vector3& thrust_b__des);

 private:
    // Parameters

    PinvMixMatrix mixerPinv;

    // Commanded

    Vector3 moment_b__des;

    Vector3 thrust_b__des;

    // parameters object - warning tolerances, physical parameters
}; // class BasicMixer NOLINT()

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_MIXER_BASIC_MIXER_H_
