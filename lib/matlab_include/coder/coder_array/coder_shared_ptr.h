/* Copyright 2019 The Mathworks, Inc. */


#pragma once
#include <iostream>
#include <assert.h>

// Custom shared intrusive pointer. Counts the number of items pointing to an object
// without storing a separate reference count. Requires the corresponding object to
// store a field called "referenceCount" as well as friend this class.

namespace coder {
template <typename T>
class shared_ptr {
  public:
    explicit shared_ptr(T* p = nullptr);
    shared_ptr(const shared_ptr<T>& sp);
    unsigned int use_count();
    T* get() const;
    bool unique() const;
    ~shared_ptr();

    shared_ptr& operator=(shared_ptr rhs);
    shared_ptr& operator=(T* rhs);

    T& operator*() const;
    T* operator->() const;
    friend void swap(shared_ptr& lhs, shared_ptr& rhs) {
        using std::swap;
        swap(lhs.fPtr, rhs.fPtr);
    }

  private:
    T* fPtr;
};
template <typename T>
shared_ptr<T>::shared_ptr(T* p)
    : fPtr(p) {
    if (fPtr != nullptr) {
        fPtr->referenceCount++;
    }
}

template <typename T>
shared_ptr<T>::shared_ptr(const shared_ptr<T>& sp)
    : fPtr(sp.fPtr) {
    if (fPtr) {
        fPtr->referenceCount++;
    }
}

template <typename T>
shared_ptr<T>& shared_ptr<T>::operator=(shared_ptr<T> rhs) {
    swap(*this, rhs);
    return *this;
}

template <typename T>
shared_ptr<T>& shared_ptr<T>::operator=(T* rhs) {
    shared_ptr<T> temp(rhs);
    *this = temp;
    return *this;
}

template <typename T>
T& shared_ptr<T>::operator*() const {
    return *fPtr;
}

template <typename T>
T* shared_ptr<T>::operator->() const {
    return fPtr;
}

// returns the number of pointers pointing to this object
template <typename T>
unsigned int shared_ptr<T>::use_count() {
    return fPtr->referenceCount;
}

// returns the object
template <typename T>
T* shared_ptr<T>::get() const {
    return fPtr;
}

// checks whether this is the only pointer pointing to this object
template <typename T>
bool shared_ptr<T>::unique() const {
    return fPtr->referenceCount == 1;
}

// if there are no more pointers pointing to this object, delete the object
template <typename T>
shared_ptr<T>::~shared_ptr() {
    if (fPtr != nullptr) {
        fPtr->referenceCount--;
        if (fPtr->referenceCount == 0) {
            delete fPtr;
        }
    }
}

template <typename T>
shared_ptr<T> make_shared() {
    return shared_ptr<T>(new T());
}
} // namespace coder