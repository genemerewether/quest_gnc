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

#include "quest_gnc/mixer/wrench_mixer.h"
#include "quest_gnc/utils/common.h"

#include "Eigen/Geometry"

#include <iostream>

#include <stdio.h>
#define DEBUG_PRINT(x,...)

#ifndef DEBUG_PRINT
#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif
#endif

namespace quest_gnc {
namespace multirotor {

WrenchMixer::WrenchMixer() :
    mixerPinv(),
    moment_b__des(0,0,0), thrust_b__des(0,0,0) {
}

WrenchMixer::~WrenchMixer() {
}

// ----------------------------------------------------------------------
// Parameter, model, and gain setters
// ----------------------------------------------------------------------

int WrenchMixer::
  SetMixer(MixMatrix mixer,
  	   unsigned int numActuators) {

    // TODO(mgardine) - use return code here if matrix inversion fails?

    if (numActuators > kWrenchMixerMaxActuators) {
        return -1;
    }
    
    if (numActuators < kWrenchMixerMaxActuators) {
        mixer.rightCols(kWrenchMixerMaxActuators - numActuators).setZero();
    }
    
    this->mixerPinv =
      mixer.transpose() *
      (mixer * mixer.transpose()).inverse();

    return 0;
}
  
// ----------------------------------------------------------------------
// Thrust and moment getters
// ----------------------------------------------------------------------

int WrenchMixer::
  GetRotorVelCommand(MixOutput* rotVel__comm) {
    FW_ASSERT(rotVel__comm);

    Vector6 controls;
    controls << this->thrust_b__des(0),
                this->thrust_b__des(1),
		this->thrust_b__des(2),
		this->moment_b__des(0),
		this->moment_b__des(1),
		this->moment_b__des(2);

    *rotVel__comm = this->mixerPinv * controls;
    *rotVel__comm = rotVel__comm->cwiseMax(
                      MixOutput::Zero(rotVel__comm->rows()));
    *rotVel__comm = rotVel__comm->cwiseSqrt();

    return 0;
}

// ----------------------------------------------------------------------
// Command setters
// ----------------------------------------------------------------------

int WrenchMixer::
  SetWrenchDes(const Vector3& thrust_b__des,
	       const Vector3& moment_b__des) {
    this->moment_b__des = moment_b__des;
    this->thrust_b__des = thrust_b__des;
    return 0;
}

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()
