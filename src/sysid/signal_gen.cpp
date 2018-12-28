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

#ifndef M_PI
#ifdef BUILD_DSPAL
#define M_PI 3.14159265358979323846
#endif
#endif

#ifndef FW_ASSERT
#define FW_ASSERT(cond) assert((cond))
#endif

#define DEBUG_PRINT(x,...)
//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)

namespace quest_gnc {
namespace sysid {

SignalGen::SignalGen() :
    unitAxis(0.0, 0.0, 1.0),
    dt(0.0),
    omega_i(0.0),
    omega_f(0.0),
    amplitude(0.0),
    iter(0u),
    nIter(0u),
    signalType(NONE) {
}

SignalGen::~SignalGen() {
}

// ----------------------------------------------------------------------
// Setters
// ----------------------------------------------------------------------

int SignalGen::
  SetChirp(FloatingPoint omega_i, FloatingPoint omega_f,
           FloatingPoint amplitude, unsigned int nIter,
           FloatingPoint dt) {
    if (omega_f < omega_i) {
        return -1;
    }

    this->omega_i = omega_i;
    this->omega_f = omega_f;
    this->amplitude = amplitude;
    this->nIter = nIter;
    this->dt = dt;

    this->iter = 0u;
    this->signalType = CHIRP;
    
    return 0;
}

int SignalGen::
  SetRamp(FloatingPoint amplitude, unsigned int nIter,
          FloatingPoint dt) {
    this->amplitude = amplitude;
    this->nIter = nIter;
    this->dt = dt;

    this->iter = 0u;
    this->signalType = RAMP;
    
    return 0;
}

int SignalGen::
  SetUnitAxis(const Vector3& axis) {
    FloatingPoint norm = axis.norm();
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
    GetScalar(FloatingPoint* val, FloatingPoint* dvaldt) {
    FW_ASSERT(val);
    FW_ASSERT(dvaldt);
    if (NONE == this->signalType) {
        return -1;
    }
    if (this->iter >= this->nIter) {
        this->signalType = NONE;
        return -1;
    }

    // fraction completed
    FloatingPoint frac = (FloatingPoint) this->iter / (FloatingPoint) this->nIter;
    // actual time
    FloatingPoint t = this->iter * this->dt;

    switch (this->signalType) {
        case CHIRP:
        {
            frac *= 0.5;
            FloatingPoint omega = this->omega_i + (this->omega_f - this->omega_i) * frac;
            *val = this->amplitude * sin(omega * t * 2 * M_PI);
            *dvaldt = this->amplitude * cos(omega * t * 2 * M_PI)
                      * (this->omega_i + 2 * (this->omega_f - this->omega_i) * frac);
        }
            break;
        case RAMP:
            *val = (frac < 0.5) ? (frac * 2.0 * this->amplitude)
                                : ((1.0 - frac) * 2.0 * this->amplitude);
            *dvaldt = (frac < 0.5) ? 2.0 * this->amplitude / (this->dt * this->nIter)
                                   : -2.0 * this->amplitude / (this->dt * this->nIter);
            break;
        case STEP:
            // TODO(mereweth)
            *val = 0.0;
            *dvaldt = 0.0;
            break;
        default:
            FW_ASSERT(0);
            break;
    }
    
    DEBUG_PRINT("val %f, dvaldt %f on iter %u of %u; time %f\n", *val, *dvaldt,
                this->iter, this->nIter, t);
    this->iter++;

    return 0;
}

int SignalGen::
  GetVector(Vector3* val) {
    FW_ASSERT(val);
    FloatingPoint scalar = 0.0f;
    FloatingPoint dvaldt = 0.0f;
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

    FloatingPoint scalar = 0.0f;
    FloatingPoint dvaldt = 0.0f;
    if (this->GetScalar(&scalar, &dvaldt)) {
        return -1;
    }
    *q = AngleAxis(scalar, this->unitAxis);
    *omega = this->unitAxis * dvaldt;

    return 0;
}

} // namespace sysid NOLINT()
} // namespace quest_gnc NOLINT()
