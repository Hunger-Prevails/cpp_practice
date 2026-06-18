# include "unique_pointer.hpp"

int Tracker::constructed = 0;
int Tracker::destroyed = 0;

template <typename T>
UniquePtr<T>::UniquePtr(T* ptr) : ptr_(ptr) {}

template <typename T>
UniquePtr<T>::~UniquePtr() {
    delete ptr_;
}

template <typename T>
UniquePtr<T>::UniquePtr(UniquePtr&& other) noexcept : ptr_(other.ptr_) {
    other.ptr_ = nullptr;
}

template <typename T>
UniquePtr<T>& UniquePtr<T>::operator=(UniquePtr&& other) noexcept {
    delete ptr_;

    ptr_ = other.ptr_;
    other.ptr_ = nullptr;

    return *this;
}

template <typename T>
T& UniquePtr<T>::operator*() const {
    return *ptr_;
}

template <typename T>
T* UniquePtr<T>::operator->() const {
    return ptr_;
}

template <typename T>
T* UniquePtr<T>::get() const {
    return ptr_;
}

template <typename T>
T* UniquePtr<T>::release() {
    T* temp = ptr_;
    ptr_ = nullptr;
    return temp;
}

template <typename T>
void UniquePtr<T>::reset(T* ptr) {
    delete ptr_;
    ptr_ = ptr;
}

template class UniquePtr<Tracker>;
