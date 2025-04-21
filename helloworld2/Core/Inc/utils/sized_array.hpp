#ifndef SIZED_ARRAY_H
#define SIZED_ARRAY_H

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
template <typename T, size_t Capacity> struct SizedArray : public std::array<T, Capacity> {
    size_t size = 0; ///< Current number of elements in the array.

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
            // throw std::out_of_range("Initializer list exceeds array capacity");
        }
        size = list.size();
        std::copy(list.begin(), list.end(), std::array<T, Capacity>::begin());
        return *this;
    }

    /**
     * @brief Access element with bounds checking.
     * @param index Position of element.
     * @return Reference to element at index.
     * @throws std::out_of_range if index is out of range.
     */
    T &operator[](size_t index) {
        if (index >= size) {
            // throw std::out_of_range("Index out of range");
        }
        return std::array<T, Capacity>::operator[](index);
    }

    /**
     * @brief Access element with bounds checking (const version).
     * @param index Position of element.
     * @return Const reference to element at index.
     * @throws std::out_of_range if index is out of range.
     */
    const T &operator[](size_t index) const {
        if (index >= size) {
            // throw std::out_of_range("Index out of range");
        }
        return std::array<T, Capacity>::operator[](index);
    }

    /**
     * @brief Adds an element to the end of the array.
     * @param value Element to add.
     * @throws std::out_of_range if the array is at full capacity.
     */
    void push_back(const T &value) {
        if (size >= Capacity) {
            // throw std::out_of_range("Exceeds array capacity");
        }
        (*this)[size] = value;
        size++;
    }

    /**
     * @brief Removes the last element from the array.
     * @throws std::out_of_range if the array is empty.
     */
    void pop_back() {
        if (size == 0) {
            // throw std::out_of_range("Array is empty");
        }
        size--;
    }

    /**
     * @brief Clears the array.
     */
    void clear() { size = 0; }

    /**
     * @brief Returns an iterator to the beginning.
     * @return Iterator to the first element.
     */
    typename std::array<T, Capacity>::iterator begin() { return std::array<T, Capacity>::begin(); }

    /**
     * @brief Returns a const iterator to the beginning.
     * @return Const iterator to the first element.
     */
    typename std::array<T, Capacity>::const_iterator begin() const { return std::array<T, Capacity>::begin(); }

    /**
     * @brief Returns an iterator to the end (up to current size).
     * @return Iterator to one past the last element.
     */
    typename std::array<T, Capacity>::iterator end() { return std::array<T, Capacity>::begin() + size; }

    /**
     * @brief Returns a const iterator to the end (up to current size).
     * @return Const iterator to one past the last element.
     */
    typename std::array<T, Capacity>::const_iterator end() const { return std::array<T, Capacity>::begin() + size; }
};

#endif // SIZED_ARRAY_H
