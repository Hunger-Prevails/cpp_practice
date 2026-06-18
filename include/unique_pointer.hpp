# include <optional>
# include <vector>


struct Tracker {
    static int constructed;
    static int destroyed;

    int value = 0;

    explicit Tracker(int v) : value(v) {
        ++constructed;
    }

    ~Tracker() {
        ++destroyed;
    }

    Tracker(const Tracker&) = delete;
    Tracker& operator=(const Tracker&) = delete;
};


template <typename T>
class UniquePtr {
public:
    explicit UniquePtr(T* ptr = nullptr);
    ~UniquePtr();

    UniquePtr(const UniquePtr&) = delete;
    UniquePtr& operator=(const UniquePtr&) = delete;

    UniquePtr(UniquePtr&& other) noexcept;
    UniquePtr& operator=(UniquePtr&& other) noexcept;

    T& operator*() const;
    T* operator->() const;
    T* get() const;

    T* release();
    void reset(T* ptr = nullptr);

private:
    T* ptr_;
};
