# include <memory>
# include <concepts>

# include "ring_buffer.hpp"

template <std::default_initializable T>
RingBuffer<T>::RingBuffer(std::size_t capacity) : capacity_(capacity), buffer_(capacity) {}

template <std::default_initializable T>
bool RingBuffer<T>::empty() const {
    return head_ == tail_;
}

template <std::default_initializable T>
bool RingBuffer<T>::full() const {
    return head_ + capacity_ == tail_;
}

template <std::default_initializable T>
std::size_t RingBuffer<T>::size() const {
    return tail_ - head_;
}

template <std::default_initializable T>
bool RingBuffer<T>::push(const T& value)
    requires std::assignable_from<T&, const T&> {
    if (full()) {
        return false;
    }
    buffer_[tail_ % capacity_] = value;
    ++tail_;
    return true;
}

template <std::default_initializable T>
bool RingBuffer<T>::push(T&& value)
    requires std::assignable_from<T&, T&&> {
    if (full()) {
        return false;
    }
    buffer_[tail_ % capacity_] = std::move(value);
    ++tail_;
    return true;
}

template <std::default_initializable T>
std::optional<T> RingBuffer<T>::pop() {
    if (empty()) {
        return std::nullopt;
    }
    T value = std::move(buffer_[head_ % capacity_]);
    ++head_;
    return value;
}

template class RingBuffer<int>;
template class RingBuffer<std::unique_ptr<int>>;
