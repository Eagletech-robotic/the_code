#pragma once

// Template parameters:
//   T         : value type stored in the queue
//   Capacity  : maximum number of elements (must be > 0)
//   Compare   : strict weak ordering functor (defaults to std::less<T>)

#include <array>
#include <concepts>
#include <functional>
#include <stdexcept>
#include <utility>

template <typename T, std::size_t Capacity, typename Compare = std::less<T>>
    requires(Capacity > 0) && std::strict_weak_order<Compare, T, T>
class BoundedPriorityQueue {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using reference = value_type &;
    using const_reference = const value_type &;

    static constexpr size_type capacity = Capacity;

    constexpr BoundedPriorityQueue() noexcept(std::is_nothrow_default_constructible_v<Compare>) = default;

    [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0; }
    [[nodiscard]] constexpr size_type size() const noexcept { return size_; }

    constexpr void clear() noexcept { size_ = 0; }

    [[nodiscard]] constexpr const_reference top() const { return data_[0]; }

    constexpr void push(const value_type &value) { do_push(value); }

    constexpr void push(value_type &&value) { do_push(std::move(value)); }

    template <typename... Args> constexpr void emplace(Args &&...args) {
        data_[size_] = value_type(std::forward<Args>(args)...);
        sift_up(size_++);
    }

    constexpr void pop() {
        --size_;
        if (size_ > 0) {
            data_[0] = std::move(data_[size_]);
            sift_down(0);
        }
    }

  private:
    std::array<value_type, capacity> data_{}; // zero‑initialised for constexpr
    size_type size_{0};
    Compare comp_{}; // comparator instance

    constexpr void do_push(value_type &&value) {
        data_[size_] = std::forward<value_type>(value);
        sift_up(size_++);
    }

    constexpr void sift_up(size_type idx) noexcept {
        while (idx > 0) {
            size_type parent = (idx - 1) / 2;
            if (!comp_(data_[parent], data_[idx]))
                break; // parent has higher/equal priority
            std::swap(data_[parent], data_[idx]);
            idx = parent;
        }
    }

    constexpr void sift_down(size_type idx) noexcept {
        while (true) {
            size_type left = idx * 2 + 1;
            size_type right = left + 1;
            if (left >= size_)
                break; // no children

            size_type best = left;
            if (right < size_ && comp_(data_[left], data_[right])) {
                best = right; // right child has higher priority
            }
            if (!comp_(data_[idx], data_[best]))
                break; // heap invariant satisfied
            std::swap(data_[idx], data_[best]);
            idx = best;
        }
    }
};
