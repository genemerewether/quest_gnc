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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_SYSID_SIGNAL_GEN_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_SYSID_SIGNAL_GEN_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {
namespace sysid {

class SignalGen {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SignalGen();

    ~SignalGen();

  // ----------------------------------------------------------------------
  // Setters
  // ----------------------------------------------------------------------

    int SetChirp(double omega_i, double omega_f, double amplitude,
                 unsigned int nIter, double dt);

    int SetUnitAxis(const Vector3& axis);

  // ----------------------------------------------------------------------
  // Getters
  // ----------------------------------------------------------------------

    int GetScalar(double* val, double* dvaldt);

    int GetVector(Vector3* val);

    int GetSO3(Quaternion* q, Vector3* omega);

 private:
    // Parameters

    // axis for axis-angle SO3 chirp or vector chirp
    Vector3 unitAxis;

    // Underlying chirp sinusoid

    double dt; // sec
    double omega_i; // Hz
    double omega_f; // Hz
    double amplitude;

    unsigned int iter;
    unsigned int nIter;

    // parameters object - warning tolerances, physical parameters
}; // class SignalGen NOLINT()

} // namespace sysid NOLINT()
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_SYSID_SIGNAL_GEN_H_
