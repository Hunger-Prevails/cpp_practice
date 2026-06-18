# include <optional>
# include <vector>

template <std::default_initializable T>
class RingBuffer {
protected:
    std::vector<T> buffer_;
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t capacity_ = 0;
public:
    RingBuffer(std::size_t capacity);

    bool push(const T& value)
        requires std::assignable_from<T&, const T&>;
    bool push(T&& value)
        requires std::assignable_from<T&, T&&>;

    std::optional<T> pop();

    bool empty() const;
    bool full() const;

    std::size_t size() const;
};
