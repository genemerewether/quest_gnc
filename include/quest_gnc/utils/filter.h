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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_FILTER_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_FILTER_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {

template<int MAX_FILT_SIZE>
class filter {
 public:
    filter();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingPoint execute(FloatingPoint x);

    void resetState();
    
    int setNumerator(int size, const FloatingPoint b[MAX_FILT_SIZE]);

    // NOTE(Mereweth) - first coefficient of denominator must be one
    int setDenominator(int size, const FloatingPoint a[MAX_FILT_SIZE]);

    inline int updateSingleNumCoeff(int n, FloatingPoint val);
    
    inline int updateSingleDenCoeff(int n, FloatingPoint val);

    int setPassthrough();
    
 private:
    int aSize_;
    int bSize_;
    FloatingPoint a_[MAX_FILT_SIZE]; // denominator coefficients
    FloatingPoint b_[MAX_FILT_SIZE]; // numerator coefficients

    FloatingPoint x_[(MAX_FILT_SIZE - 1) * 2]; // buffered inputs
    FloatingPoint y_[(MAX_FILT_SIZE - 1) * 2]; // buffered outputs
    
    bool initialized_;

    int histIdx_; // use buffered input / output as circular buffers

    // NOTE(mereweth) - init to steady-state for input x
    void initialize(FloatingPoint x);
};

inline int filtDenValid(const FloatingPoint* values, int size) {
    FW_ASSERT(size >= 1);

    /* NOTE(mereweth) - first denom coefficient is not used for computation;
     * just to make sure the caller understands the convention
     */
    if (values[0] < 1.0 - 1e-5 || values[0] > 1.0 + 1e-5) {
      return 1;
    } else {
      return 0;
    }
}

template <int MAX_FILT_SIZE>
filter<MAX_FILT_SIZE>::filter() {
    setPassthrough();

    resetState();
}

template <int MAX_FILT_SIZE>
FloatingPoint filter<MAX_FILT_SIZE>::execute(FloatingPoint x) {
    const FloatingPoint* aCoeff = &a_[1];
    const FloatingPoint* bCoeff = &b_[1];
    const FloatingPoint* xElem = &x_[histIdx_];
    const FloatingPoint* yElem = &y_[histIdx_];

    if (!initialized_) {
      initialize(x);
      initialized_ = true;
    }

    // NOTE(mereweth) - we start from the first coefficients below, so handle the first numerator coeff here
    FloatingPoint bSum = b_[0] * x;
    FloatingPoint aSum = 0;

    // iter 0: [ x[n-1] x[n-2] x[n-1] x[n-2] ], use elems 0, 1; set elems 1, 3
    // iter 1: [ x[n-2] x[n-1] x[n-2] x[n-1] ], use elems 1, 2; set elems 0, 2
    // repeat cycle:
    // iter 2: [ x[n-1] x[n-2] x[n-1] x[n-2] ], use elems 0, 1; set elems 1, 3

    /* NOTE(mereweth) - the last element never gets used; rather it is there
     * so we can use the same logic for setting indices for every iteration
     * in the cycle above
     */

    for (int i = 0; i < MAX_FILT_SIZE - 1; i++) {
        // NOTE(mereweth) - index through the filter coefficients and history
	bSum += (*(bCoeff++)) * (*(xElem++));
	aSum += (*(aCoeff++)) * (*(yElem++));
    }

    // y[n] = sum{k=1,N}(b_k * x[n-k]) - sum{k=1,M}(a_k * y[n-k])
    const FloatingPoint y = bSum - aSum;

    if (--histIdx_ < 0) {
        histIdx_ = MAX_FILT_SIZE - 2;
    }

    /* NOTE(mereweth) - can't exceed array bounds: histIdx_ max val is MAX_FILT_SIZE - 2,
     * so histIdxTopHalf max val is (MAX_FILT_SIZE - 1) * 2 - 1, one less than length of array
     */
    const int histIdxTopHalf = histIdx_ + MAX_FILT_SIZE - 1;

    x_[histIdx_] = x_[histIdxTopHalf] = x;
    y_[histIdx_] = y_[histIdxTopHalf] = y;

    return y;
}

template <int MAX_FILT_SIZE>
void filter<MAX_FILT_SIZE>::resetState() {
    initialized_ = false;
    histIdx_ = 0;
}

template <int MAX_FILT_SIZE>
int filter<MAX_FILT_SIZE>::setNumerator(int size,
                                        const FloatingPoint b[MAX_FILT_SIZE]) {
    int i;

    if (!(size > 0 && size <= MAX_FILT_SIZE)) {
	return 1;
    }

    bSize_ = size;
    for (i = 0; i < bSize_; i++) {
	b_[i] = b[i];
    }

    // NOTE(mereweth) - zero the unused coefficients
    for (i = bSize_; i < MAX_FILT_SIZE; i++) {
      b_[i] = 0.0;
    }

    resetState();

    return 0;
}



template <int MAX_FILT_SIZE>
int filter<MAX_FILT_SIZE>::setDenominator(int size,
                                          const FloatingPoint a[MAX_FILT_SIZE]) {
    int i;

    if (!(size > 0 && size <= MAX_FILT_SIZE))
        return 1;

    if (filtDenValid(a, size) != 0) {
        return 2;
    }

    aSize_ = size;
    for (i = 0; i < aSize_; i++) {
        a_[i] = a[i];
    }

    // NOTE(mereweth) - zero the unused coefficients  
    for (i = aSize_; i < MAX_FILT_SIZE; i++) {
        a_[i] = 0.0;
    }

    resetState();

    return 0;
}

template <int MAX_FILT_SIZE>
int filter<MAX_FILT_SIZE>::updateSingleNumCoeff(int n, FloatingPoint val) {
    if (n >= bSize_) {
        return 1;
    }

    b_[n] = val;

    return 0;
}

template <int MAX_FILT_SIZE>
int filter<MAX_FILT_SIZE>::updateSingleDenCoeff(int n, FloatingPoint val) {
    if (n >= aSize_) {
        return 1;
    }

    if (n == 0 && (val < 1.0 - 1e-5 || val > 1.0 + 1e-5)) {
        return 1;
    }

    a_[n] = val;

    return 0;
}

template <int MAX_FILT_SIZE>
int filter<MAX_FILT_SIZE>::setPassthrough() {
  const F64 b[1] = {1.0};
  const F64 a[1] = {1.0};

  return setNumerator(1, b) + setDenominator(1, a);
}
 
template <int MAX_FILT_SIZE>
void filter<MAX_FILT_SIZE>::initialize(FloatingPoint x) {
    FloatingPoint y_ss = 0.0;

    FloatingPoint bSum = 0.0;
    FloatingPoint aSum = 0.0;

    for (int i = 0; i < bSize_; i++) {
        bSum += b_[i];
    }

    for (int i = 0; i < aSize_; i++) {
        aSum += a_[i];
    }

    // If this is not true, filter parameters are invalid; output history will
    // be set to zero.
    if (fabs(aSum) > 1e-5) {
        y_ss = bSum / aSum * x;
    }

    for (int i = 0; i < (MAX_FILT_SIZE - 1) * 2; i++) {
        x_[i] = x;
        y_[i] = y_ss;
    }
}
 
} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_FILTER_H_
