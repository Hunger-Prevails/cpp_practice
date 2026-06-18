# include <iostream>
# include <cassert>
# include <optional>
# include <string>
# include <utility>
# include <cxxopts.hpp>
# include <type_traits>

# include "ring_buffer.hpp"
# include "unique_pointer.hpp"

namespace fs = std::filesystem;

void test_empty_buffer() {
    auto rb = RingBuffer<int>(3);

    assert(rb.empty());
    assert(!rb.full());
    assert(rb.size() == 0);

    auto value = rb.pop();
    assert(!value.has_value());
}

void test_push_pop_basic_fifo() {
    auto rb = RingBuffer<int>(3);

    assert(rb.push(10));
    assert(rb.push(20));

    assert(!rb.empty());
    assert(!rb.full());
    assert(rb.size() == 2);

    auto a = rb.pop();
    auto b = rb.pop();

    assert(a.has_value());
    assert(b.has_value());
    assert(*a == 10);
    assert(*b == 20);

    assert(rb.empty());
    assert(rb.size() == 0);
}

void test_full_buffer_rejects_new_data() {
    auto rb = RingBuffer<int>(3);

    assert(rb.push(1));
    assert(rb.push(2));
    assert(rb.push(3));

    assert(rb.full());
    assert(rb.size() == 3);

    // This test assumes non-overwriting behavior.
    assert(!rb.push(4));

    auto a = rb.pop();
    auto b = rb.pop();
    auto c = rb.pop();
    auto d = rb.pop();

    assert(a.has_value() && *a == 1);
    assert(b.has_value() && *b == 2);
    assert(c.has_value() && *c == 3);
    assert(!d.has_value());

    assert(rb.empty());
}

void test_wraparound_behavior() {
    auto rb = RingBuffer<int>(3);

    assert(rb.push(1));
    assert(rb.push(2));
    assert(rb.push(3));

    assert(rb.full());

    auto a = rb.pop();
    assert(a.has_value() && *a == 1);

    assert(!rb.full());
    assert(rb.size() == 2);

    // This should reuse the freed slot at the beginning of the array.
    assert(rb.push(4));

    assert(rb.full());
    assert(rb.size() == 3);

    auto b = rb.pop();
    auto c = rb.pop();
    auto d = rb.pop();

    assert(b.has_value() && *b == 2);
    assert(c.has_value() && *c == 3);
    assert(d.has_value() && *d == 4);

    assert(rb.empty());
}

void test_multiple_wraparounds() {
    auto rb = RingBuffer<int>(4);

    for (int i = 0; i < 100; ++i) {
        assert(rb.push(i));
        auto value = rb.pop();

        assert(value.has_value());
        assert(*value == i);
        assert(rb.empty());
        assert(rb.size() == 0);
    }
}

void test_interleaved_operations() {
    auto rb = RingBuffer<int>(5);

    assert(rb.push(1));
    assert(rb.push(2));
    assert(rb.push(3));

    auto a = rb.pop();
    assert(a.has_value() && *a == 1);

    assert(rb.push(4));
    assert(rb.push(5));

    auto b = rb.pop();
    auto c = rb.pop();

    assert(b.has_value() && *b == 2);
    assert(c.has_value() && *c == 3);

    assert(rb.push(6));
    assert(rb.push(7));
    assert(rb.push(8));

    assert(rb.full());

    auto d = rb.pop();
    auto e = rb.pop();
    auto f = rb.pop();
    auto g = rb.pop();
    auto h = rb.pop();

    assert(d.has_value() && *d == 4);
    assert(e.has_value() && *e == 5);
    assert(f.has_value() && *f == 6);
    assert(g.has_value() && *g == 7);
    assert(h.has_value() && *h == 8);

    assert(rb.empty());
}

void test_move_only_type() {
    auto rb = RingBuffer<std::unique_ptr<int>>(2);

    auto p1 = std::make_unique<int>(10);
    auto p2 = std::make_unique<int>(20);

    assert(rb.push(std::move(p1)));
    assert(rb.push(std::move(p2)));

    assert(p1 == nullptr);
    assert(p2 == nullptr);
    assert(rb.full());

    auto a = rb.pop();
    auto b = rb.pop();

    assert(a.has_value());
    assert(b.has_value());

    assert(*a != nullptr);
    assert(*b != nullptr);

    assert(**a == 10);
    assert(**b == 20);

    assert(rb.empty());
}


int main() {
    test_empty_buffer();
    test_push_pop_basic_fifo();
    test_full_buffer_rejects_new_data();
    test_wraparound_behavior();
    test_multiple_wraparounds();
    test_interleaved_operations();
    test_move_only_type();

    std::cout << "All RingBuffer tests passed." << std::endl;

    static_assert(!std::is_copy_constructible_v<UniquePtr<Tracker>>);
    static_assert(!std::is_copy_assignable_v<UniquePtr<Tracker>>);
    static_assert(std::is_move_constructible_v<UniquePtr<Tracker>>);
    static_assert(std::is_move_assignable_v<UniquePtr<Tracker>>);

    {
        UniquePtr<Tracker> p(new Tracker(42));

        assert(p.get() != nullptr);
        assert((*p).value == 42);
        assert(p->value == 42);
        assert(Tracker::constructed == 1);
        assert(Tracker::destroyed == 0);
    }

    assert(Tracker::destroyed == 1);

    {
        UniquePtr<Tracker> p1(new Tracker(10));
        UniquePtr<Tracker> p2(std::move(p1));

        assert(p1.get() == nullptr);
        assert(p2.get() != nullptr);
        assert(p2->value == 10);
    }

    assert(Tracker::destroyed == 2);

    {
        UniquePtr<Tracker> p1(new Tracker(100));
        UniquePtr<Tracker> p2(new Tracker(200));

        p2 = std::move(p1);

        assert(p1.get() == nullptr);
        assert(p2.get() != nullptr);
        assert(p2->value == 100);

        // The old Tracker(200) should have been destroyed during move assignment.
        assert(Tracker::destroyed == 3);
    }

    assert(Tracker::destroyed == 4);

    {
        UniquePtr<Tracker> p(new Tracker(7));

        Tracker* raw = p.release();

        assert(p.get() == nullptr);
        assert(raw != nullptr);
        assert(raw->value == 7);

        // release() transfers ownership out, so UniquePtr should not delete it.
        delete raw;
    }

    assert(Tracker::destroyed == 5);

    {
        UniquePtr<Tracker> p(new Tracker(1));
        p.reset(new Tracker(2));

        // reset should destroy the old object.
        assert(Tracker::destroyed == 6);
        assert(p.get() != nullptr);
        assert(p->value == 2);

        p.reset();

        assert(p.get() == nullptr);
        assert(Tracker::destroyed == 7);
    }

    std::cout << "All UniquePtr tests passed." << std::endl;

    return 0;
}
