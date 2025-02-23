// AI for a robot playing coupe de france de robotique, using gradient descent essentially. Since it
// is robotics, we are not allowed any memory allocation.

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "iot01A/top_driver.h"

template <typename T, size_t Capacity>
struct SizedArray : public std::array<T, Capacity> {
    size_t size = 0;

    SizedArray() = default;

    SizedArray(std::initializer_list<T> list) { *this = list; }

    SizedArray& operator=(std::initializer_list<T> list) {
        if (list.size() > Capacity) {
            throw std::out_of_range("Initializer list exceeds array capacity");
        }
        size = list.size();
        std::copy(list.begin(), list.end(), std::array<T, Capacity>::begin());
        return *this;
    }

    T& operator[](size_t index) {
        if (index >= size) {
            throw std::out_of_range("Index out of range");
        }
        return std::array<T, Capacity>::operator[](index);
    }

    const T& operator[](size_t index) const {
        if (index >= size) {
            throw std::out_of_range("Index out of range");
        }
        return std::array<T, Capacity>::operator[](index);
    }

    void push_back(const T& value) {
        if (size >= Capacity) {
            throw std::out_of_range("Exceeds array capacity");
        }
        (*this)[size] = value;
        size++;
    }

    void pop_back() {
        if (size == 0) {
            throw std::out_of_range("Array is empty");
        }
        size--;
    }

    void clear() { size = 0; }

    typename std::array<T, Capacity>::iterator begin() { return std::array<T, Capacity>::begin(); }

    typename std::array<T, Capacity>::const_iterator begin() const {
        return std::array<T, Capacity>::begin();
    }

    typename std::array<T, Capacity>::iterator end() {
        return std::array<T, Capacity>::begin() + size;
    }

    typename std::array<T, Capacity>::const_iterator end() const {
        return std::array<T, Capacity>::begin() + size;
    }
};

constexpr uint8_t SQUARE_SIZE_CM = 4;

struct GameEntity {
    uint16_t x;
    uint16_t y;
    uint16_t orientation_degrees;

    GameEntity() = default;

    GameEntity(uint16_t x, uint16_t y, uint16_t orientation_degrees) {
        if (x >= 300 || y >= 200) {
            throw std::out_of_range("Coordinates out of range");
        }
        this->x = x;
        this->y = y;
        this->orientation_degrees = orientation_degrees;
    }
};

constexpr uint16_t BLEACHER_INFLUENCE_SIZE = 150;
class Bleacher : public GameEntity {
   public:
    const std::array<std::array<float, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>,
                     BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>&
    potential_field() {
        const int size = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
        static std::array<std::array<float, size>, size> field;

        float center_x = size / 2.0;
        float center_y = size / 2.0;

        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                float dx = std::abs(x - center_x);
                float dy = std::abs(y - center_y);
                field[x][y] = potential_function(dx, dy);
            }
        }

        return field;
    }

   private:
    float potential_function(float dx, float dy) {
        int scale = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
        float clamped_dx = dx / scale * M_PI;
        float clamped_dy = dy / scale * M_PI;
        float value = -std::exp(-clamped_dx - clamped_dy) /
                      (1 + clamped_dx * clamped_dx + clamped_dy * clamped_dy);
        return value * scale / M_PI;
    }
};

float potential_field[300 / SQUARE_SIZE_CM][200 / SQUARE_SIZE_CM]{};

SizedArray<Bleacher, 40> bleachers;
SizedArray<GameEntity, 40> can;

extern "C" {

void visualize_potential_field(float potential_field[300 / SQUARE_SIZE_CM][200 / SQUARE_SIZE_CM],
                               size_t width, size_t height) {
    int colors[] = {
        17, 19, 20, 26, 32, 38, 46, 82, 118, 154, 190, 226, 214, 208, 202, 196, 160, 124, 88, 52,
    };

    float minValue = potential_field[0][0];
    float maxValue = potential_field[0][0];

    for (size_t x = 0; x < width; ++x) {
        for (size_t y = 0; y < height; ++y) {
            minValue = std::min(minValue, potential_field[x][y]);
            maxValue = std::max(maxValue, potential_field[x][y]);
        }
    }

    std::string log = "";

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            float normalized = (potential_field[x][y] - minValue) / (maxValue - minValue);
            int index = (int)(normalized * (sizeof(colors) / sizeof(colors[0]) - 1));
            log += "\033[48;5;" + std::to_string(colors[index]) + "m  \033[0m";
        }
        log += "\n";
    }

    printf("%s", log.c_str());
    printf("Min: %f, Max: %f\n", minValue, maxValue);
}

void thibault_top_init(config_t* config) {
    /*bleachers = {
        Bleacher{{100, 120, 0}}, Bleacher{{20, 100, 0}},  Bleacher{{228, 100, 0}},
        Bleacher{{150, 50, 0}},  Bleacher{{150, 180, 0}}, Bleacher{{50, 50, 0}},
        Bleacher{{100, 0, 0}},
    };*/
    bleachers = {
        Bleacher{{100, 120, 0}},

    };

    float* first = &potential_field[0][0];
    std::fill(first, first + (300 / SQUARE_SIZE_CM) * (200 / SQUARE_SIZE_CM), 0);

    for (auto& bleacher : bleachers) {
        auto& field = bleacher.potential_field();
        for (uint16_t x = 0; x < field.size(); x++) {
            for (uint16_t y = 0; y < field.size(); y++) {
                uint16_t x_index = x + bleacher.x / SQUARE_SIZE_CM - field.size() / 2;
                uint16_t y_index = y + bleacher.y / SQUARE_SIZE_CM - field[0].size() / 2;

                if (x_index >= 300 / SQUARE_SIZE_CM || y_index >= 200 / SQUARE_SIZE_CM) {
                    continue;
                }

                potential_field[x_index][y_index] += field[x][y];
            }
        }
    }

#ifdef STANDALONE
    visualize_potential_field(potential_field, 300 / SQUARE_SIZE_CM, 200 / SQUARE_SIZE_CM);
#endif
}

void thibault_top_step(config_t* config, input_t* input, output_t* output) {
    uint16_t index_x = input->x_mm / 10.0 / SQUARE_SIZE_CM;
    uint16_t index_y = input->y_mm / 10.0 / SQUARE_SIZE_CM;

    if (index_x >= 300 / SQUARE_SIZE_CM || index_y >= 200 / SQUARE_SIZE_CM) {
        throw std::out_of_range("Coordinates out of range");
    }

    float potentials[8] = {
        potential_field[index_x][index_y - 1], potential_field[index_x + 1][index_y - 1],
        potential_field[index_x + 1][index_y], potential_field[index_x + 1][index_y + 1],
        potential_field[index_x][index_y + 1], potential_field[index_x - 1][index_y + 1],
        potential_field[index_x - 1][index_y], potential_field[index_x - 1][index_y - 1],
    };

    std::pair<int, float> best_potential = std::make_pair(0, potentials[0]);
    for (int i = 0; i < 8; i++) {
        printf("Potential %d: %f\n", i, potentials[i]);
        if (potentials[i] < best_potential.second) {
            best_potential = {i, potentials[i]};
        }
    }

    float target_angle_deg = best_potential.first * 45;

    float angle_diff = std::fmod(target_angle_deg - input->orientation_degrees, 360);
    if (angle_diff < 0) angle_diff += 360;

    if (angle_diff <= 5 || angle_diff >= 355) {
        output->vitesse1_ratio = 0.5;
        output->vitesse2_ratio = 0.5;
    } else if (angle_diff <= 180) {
        output->vitesse1_ratio = 0.7;
        output->vitesse2_ratio = 0.3;
    } else {
        output->vitesse1_ratio = 0.3;
        output->vitesse2_ratio = 0.7;
    }

    printf("X: %d Y: %d\n", index_x, index_y);
    printf("Current potential: %f\n", potential_field[index_x][index_y]);
    printf("Best potential: %f\n", best_potential.second);
    printf("Target angle: %f\n", target_angle_deg);
    printf("Angle diff: %f\n", angle_diff);
}

#ifdef STANDALONE
int main() {
    config_t config;
    input_t input;
    output_t output;

    thibault_top_init(&config);
    return 0;
}
#endif

}  // extern "C"