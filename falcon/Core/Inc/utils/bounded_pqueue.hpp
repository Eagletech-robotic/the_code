#pragma once
#include <array>
#include <concepts>
#include <functional>
#include <limits>
#include <utility>

template <typename T, std::size_t Capacity, typename Compare = std::less<T>>
requires(Capacity > 0) && std::strict_weak_order<Compare, T, T> class BoundedPriorityQueue : Compare {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using reference = value_type &;
    using const_reference = const value_type &;

    static_assert(Capacity <= (std::numeric_limits<size_type>::max() >> 1), "size_type too small for this capacity");

    static constexpr size_type capacity = Capacity;

    constexpr BoundedPriorityQueue() noexcept = default;

    [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0; }
    [[nodiscard]] constexpr size_type size() const noexcept { return size_; }

    constexpr void clear() noexcept { size_ = 0; }

    [[nodiscard]] constexpr const_reference top() const { return data_[0]; }

    constexpr void push(const value_type &v) { emplace(v); }
    constexpr void push(value_type &&v) { emplace(std::move(v)); }

    template <typename... Args> constexpr void emplace(Args &&...args) {
        // Create the hole at the end, then sift it up.
        value_type tmp(std::forward<Args>(args)...);
        sift_up_hole(size_, std::move(tmp));
        ++size_;
    }

    constexpr void pop() {
        // Move last element to root hole, then sift it down.
        value_type tmp(std::move(data_[--size_]));
        sift_down_hole(0, std::move(tmp));
    }

  private:
    std::array<value_type, capacity> data_{};
    size_type size_{0};

    // Convenience: expose comparator as a function-call operator
    [[nodiscard]] constexpr const Compare &cmp() const noexcept { return *this; }

    constexpr void sift_up_hole(size_type hole, value_type &&val) noexcept {
        const auto &less = static_cast<const Compare &>(*this);
        while (hole) {
            size_type parent = (hole - 1) >> 1;
            if (!less(data_[parent], val)) [[likely]]
                break;
            data_[hole] = std::move(data_[parent]); // pull parent down
            hole = parent;
        }
        data_[hole] = std::move(val); // drop into place
    }

    constexpr void sift_down_hole(size_type hole, value_type &&val) noexcept {
        size_type half = size_ >> 1; // last internal node
        while (hole < half) {
            size_type left = (hole << 1) + 1;
            size_type right = left + 1;
            size_type best = (right < size_ && cmp()(data_[left], data_[right])) ? right : left;
            if (!cmp()(val, data_[best])) [[likely]]
                break;
            data_[hole] = std::move(data_[best]); // push child up
            hole = best;
        }
        data_[hole] = std::move(val);
    }
};
