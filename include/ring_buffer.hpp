# include <optional>
# include <vector>

class RingBuffer {
protected:
    std::vector<int> buffer_;
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t capacity_ = 0;
public:
    RingBuffer(std::size_t capacity) : capacity_(capacity) {}

    bool push(const int& value);
    bool push(int&& value);

    std::optional<int> pop();

    bool empty() const;
    bool full() const;

    std::size_t size() const;
};
