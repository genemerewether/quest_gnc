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

#ifndef QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_RINGBUFFER_H_
#define QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_RINGBUFFER_H_

#include <Eigen/Eigen>

#include "quest_gnc/utils/common.h"

namespace quest_gnc {

template<typename T, size_t CAPACITY>
class ringbuffer {
 public:
    ringbuffer() :
        head_(0u),
        tail_(0u),
        size_(0u),
        buf()
    {
        COMPILE_TIME_ASSERT(CAPACITY > 0, RINGBUFFER_ZERO_CAPACITY);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int queue(const T* item) {
        FW_ASSERT(item);

        buf[head_] = *item;
        head_ = (head_ + 1) % CAPACITY;

        if (size_ < CAPACITY) {
            size_++;
            return 0;
        }
        else {
            tail_ = (tail_ + 1) % CAPACITY;
            return -1;
        }
    }

    int queueAndGetPtr(T** itemPtr) {
        FW_ASSERT(itemPtr);

        *itemPtr = &buf[head_];
        head_ = (head_ + 1) % CAPACITY;

        if (size_ < CAPACITY) {
            size_++;
            return 0;
        }
        else {
            tail_ = (tail_ + 1) % CAPACITY;
            return -1;
        }
    }

    int dequeue(T* item) {
        FW_ASSERT(item);
        if (0 == size_) {
            return -1;
        }

        *item = buf[tail_];
        size_--;
        tail_ = (tail_ + 1) % CAPACITY;
        return 0;
    }

    const T* getLastIn() const {
        if (size_ > 0) {
            return &buf[(tail_ + size_ - 1) % CAPACITY];
        }
        else {
            return NULL;
        }
    }

    const T* getFirstIn() const {
        if (size_ > 0) {
            return &buf[tail_];
        }
        else {
            return NULL;
        }
    }

    const T* get(unsigned int i) const {
        if ((i >= 0) && (i < size_)) {
            return &buf[(tail_ + i) % CAPACITY];
        }
        else {
            return NULL;
        }
    }

    int remove() {
        if (0 == size_) {
            return -1;
        }

        size_--;
        tail_ = (tail_ + 1) % CAPACITY;
        return 0;
    }

    void reset() {
        head_ = 0u;
        tail_ = 0u;
        size_ = 0u;
    }

    unsigned int size() {
        return size_;
    }

    unsigned int capacity() {
        return CAPACITY;
    }

 private:
    unsigned int head_;
    unsigned int tail_;
    unsigned int size_;
    T buf[CAPACITY];
};

} // namespace quest_gnc NOLINT()

#endif  // QUEST_GNC_INCLUDE_QUEST_GNC_UTILS_RINGBUFFER_H_
