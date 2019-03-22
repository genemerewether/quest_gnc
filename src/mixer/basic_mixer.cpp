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

#include "quest_gnc/mixer/basic_mixer.h"

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

BasicMixer::BasicMixer() :
    mixerPinv(),
    moment_b__des(0,0,0), thrust_b__des(0,0,0) {
}

BasicMixer::~BasicMixer() {
}

// ----------------------------------------------------------------------
// Parameter, model, and gain setters
// ----------------------------------------------------------------------

int BasicMixer::
  SetMixer(MixMatrix mixer,
  	   unsigned int numActuators) {

    // TODO(mgardine) - use return code here if matrix inversion fails?

    if (numActuators > kBasicMixerMaxActuators) {
        return -1;
    }
    
    if (numActuators < kBasicMixerMaxActuators) {
        mixer.rightCols(kBasicMixerMaxActuators - numActuators).setZero();
    }
    
    this->mixerPinv =
      mixer.transpose() *
      (mixer * mixer.transpose()).inverse();

    return 0;
}
  
// ----------------------------------------------------------------------
// Thrust and moment getters
// ----------------------------------------------------------------------

int BasicMixer::
  GetRotorVelCommand(MixOutput* rotVel__comm) {
    FW_ASSERT(rotVel__comm);

    const Eigen::Vector4d controls(this->moment_b__des(0),
                             this->moment_b__des(1),
                             this->moment_b__des(2),
                             this->thrust_b__des(2));

    *rotVel__comm = this->mixerPinv * controls;
    *rotVel__comm = rotVel__comm->cwiseMax(
                      MixOutput::Zero(rotVel__comm->rows()));
    *rotVel__comm = rotVel__comm->cwiseSqrt();

    return 0;
}

// ----------------------------------------------------------------------
// Command setters
// ----------------------------------------------------------------------

int BasicMixer::
  SetTorqueThrustDes(const Vector3& moment_b__des,
                     const Vector3& thrust_b__des) {
    this->moment_b__des = moment_b__des;
    this->thrust_b__des = thrust_b__des;
    return 0;
}

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()
