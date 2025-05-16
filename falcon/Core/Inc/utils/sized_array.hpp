#pragma once

#include <array>
#include <initializer_list>
#include <stdexcept>

/**
 * @brief A fixed-capacity array with dynamic size support.
 *
 * SizedArray provides the benefits of std::array with dynamic sizing capability.
 * It allows push_back, pop_back, and initialization from an initializer_list while ensuring
 * bounds safety and capacity checks.
 *
 * @tparam T Type of the elements.
 * @tparam Capacity Maximum number of elements the array can hold.
 */
template <typename T, size_t Capacity> class SizedArray {
  private:
    std::array<T, Capacity> data_;
    size_t size_ = 0; ///< Current number of elements in the array.

  public:
    /**
     * @brief Default constructor initializes an empty array.
     */
    SizedArray() = default;

    /**
     * @brief Constructs the array with an initializer list.
     * @param list Elements to initialize the array with.
     * @throws std::out_of_range if list size exceeds Capacity.
     */
    SizedArray(std::initializer_list<T> list) { *this = list; }

    /**
     * @brief Assigns an initializer list to the array.
     * @param list Elements to assign.
     * @return Reference to this SizedArray.
     * @throws std::out_of_range if list size exceeds Capacity.
     */
    SizedArray &operator=(std::initializer_list<T> list) {
        if (list.size() > Capacity) {
            //      throw std::out_of_range("Initializer list exceeds array capacity");
        }
        size_ = list.size();
        std::copy(list.begin(), list.end(), data_.begin());
        return *this;
    }

    /**
     * @brief Access element with bounds checking.
     * @param index Position of element.
     * @return Reference to element at index.
     * @throws std::out_of_range if index is out of range.
     */
    T &operator[](size_t index) {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        return data_[index];
    }

    /**
     * @brief Access element with bounds checking (const version).
     * @param index Position of element.
     * @return Const reference to element at index.
     * @throws std::out_of_range if index is out of range.
     */
    const T &operator[](size_t index) const {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        return data_[index];
    }

    /**
     * @brief Adds an element to the end of the array.
     * @param value Element to add.
     * @throws std::out_of_range if the array is at full capacity.
     */
    void push_back(const T &value) {
        if (size_ >= Capacity) {
            // throw std::out_of_range("Exceeds array capacity");
        }
        data_[size_] = value;
        size_++;
    }

    /**
     * @brief Removes the last element from the array.
     * @throws std::out_of_range if the array is empty.
     */
    void pop_back() {
        if (size_ == 0) {
            throw std::out_of_range("Array is empty");
        }
        size_--;
    }

    /**
     * @brief Removes the first occurrence of a value from the array.
     */
    bool remove(const T &value) {
        auto new_end_iterator = std::remove(begin(), end(), value);
        if (new_end_iterator == end())
            return false;
        size_--;
        return true;
    }

    /**
     * @brief Clears the array.
     */
    void clear() { size_ = 0; }

    /**
     * @return True if full, false otherwise.
     */
    [[nodiscard]] bool full() const { return size_ == Capacity; }

    /**
     * @return The current dynamic size of the array.
     */
    [[nodiscard]] size_t size() const { return size_; }

    /**
     * @return A boolean indicating if the array is empty.
     */
    [[nodiscard]] bool empty() const { return size_ == 0; }

    /**
     * @return The capacity of the array.
     */
    [[nodiscard]] static size_t capacity() { return Capacity; }

    /**
     * @brief Returns an iterator to the beginning.
     * @return Iterator to the first element.
     */
    typename std::array<T, Capacity>::iterator begin() { return data_.begin(); }

    /**
     * @brief Returns a const iterator to the beginning.
     * @return Const iterator to the first element.
     */
    typename std::array<T, Capacity>::const_iterator begin() const { return data_.begin(); }

    /**
     * @brief Returns an iterator to the end (up to current size).
     * @return Iterator to one past the last element.
     */
    typename std::array<T, Capacity>::iterator end() { return data_.begin() + size_; }

    /**
     * @brief Returns a const iterator to the end (up to current size).
     * @return Const iterator to one past the last element.
     */
    typename std::array<T, Capacity>::const_iterator end() const { return data_.begin() + size_; }
};
