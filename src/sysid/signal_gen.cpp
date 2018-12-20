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

#include "quest_gnc/sysid/signal_gen.h"

#include "Eigen/Geometry"
#include <math.h>

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

#define DEBUG_PRINT(x,...)
//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)

namespace quest_gnc {
namespace multirotor {

SignalGen::SignalGen() :
    unitAxis(0.0, 0.0, 1.0),
    dt(0.0),
    omega_i(0.0),
    omega_f(0.0),
    amplitude(0.0),
    phase(0.0),
    iter(0u),
    nIter(0u) {
}

SignalGen::~SignalGen() {
}

// ----------------------------------------------------------------------
// Setters
// ----------------------------------------------------------------------

int SignalGen::
  SetChirp(double omega_i, double omega_f, double amplitude,
           unsigned int nIter, double dt) {
    if (omega_f < omega_i) {
        return -1;
    }

    this->omega_i = omega_i;
    this->omega_f = omega_f;
    this->amplitude = amplitude;
    this->nIter = nIter;
    this->dt = dt;

    this->iter = 0u;
    this->phase = 0.0f;

    return 0;
}

int SignalGen::
  SetUnitAxis(const Vector3& axis) {
    double norm = axis.norm();
    if (norm < 1e-6) {
        return -1;
    }

    this->unitAxis = axis / norm;

    return 0;
}

// ----------------------------------------------------------------------
// Getters
// ----------------------------------------------------------------------

int SignalGen::
    GetScalar(double* val, double* dvaldt) {
    FW_ASSERT(val);
    FW_ASSERT(dvaldt);
    if (iter >= nIter) {
        return -1;
    }

    *val = this->amplitude * sin(this->phase);
    *dvaldt = 2.0 * this->amplitude * cos(this->phase) * this->iter * this->dt;
    double omega = this->omega_i + (this->omega_f - this->omega_i)
                   * (double) this->iter / (double) this->nIter;

    this->phase = fmod(this->phase + omega, 2.0 * M_PI) ;
    iter++;

    return 0;
}

int SignalGen::
  GetVector(Vector3* val) {
    FW_ASSERT(val);
    double scalar = 0.0f;
    double dvaldt = 0.0f;
    if (this->GetScalar(&scalar, &dvaldt)) {
        return -1;
    }

    *val = scalar * this->unitAxis;

    return 0;
}

int SignalGen::
  GetSO3(Quaternion* q, Vector3* omega) {
    FW_ASSERT(q);
    FW_ASSERT(omega);

    double scalar = 0.0f;
    double dvaldt = 0.0f;
    if (this->GetScalar(&scalar, &dvaldt)) {
        return -1;
    }
    *q = AngleAxis(scalar, this->unitAxis);
    *omega = this->unitAxis * dvaldt;

    return 0;
}

} // namespace multirotor NOLINT()
} // namespace quest_gnc NOLINT()
