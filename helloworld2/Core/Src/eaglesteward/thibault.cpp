// AI for a robot playing coupe de france de robotique, using gradient descent essentially. Since it
// is robotics, we are not allowed any memory allocation.

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "iot01A/top_driver.h"

template <typename T, size_t Capacity>
struct SizedArray : public std::array<T, Capacity> {
    size_t size = 0;

    SizedArray() = default;

    SizedArray(std::initializer_list<T> list) {
        if (list.size() > Capacity) {
            throw std::out_of_range("Initializer list exceeds array capacity");
        }
        size = list.size();
        std::copy(list.begin(), list.end(), std::array<T, Capacity>::begin());
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

class GameEntity {
   public:
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
    const std::array<std::array<int16_t, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>,
                     BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>&
    potentialField() {
        const int size = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
        static std::array<std::array<int16_t, size>, size> field;

        float center_x = size / 2.0;
        float center_y = size / 2.0;

        for (uint16_t x = 0; x < size; x++) {
            for (uint16_t y = 0; y < size; y++) {
                float dx = std::abs(x - center_x);
                float dy = std::abs(y - center_y);
                field[x][y] = potentialFunction(dx, dy);
            }
        }

        return field;
    }

   private:
    float potentialFunction(float dx, float dy) {
        float clamped_dx = dx / (BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM) * M_PI;
        float clamped_dy = dy / (BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM) * M_PI;
        float value = -1 / (1 + clamped_dx * clamped_dx + clamped_dy * clamped_dy) *
                      (std::exp(-clamped_dx - clamped_dy));
        return value * (BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM) / M_PI;
    }
};

int16_t potential_field[300 / SQUARE_SIZE_CM][200 / SQUARE_SIZE_CM]{};

SizedArray<Bleacher, 40> bleachers = {
    Bleacher{{100, 120, 0}}, Bleacher{{20, 100, 0}},  Bleacher{{228, 100, 0}},
    Bleacher{{150, 50, 0}},  Bleacher{{150, 180, 0}}, Bleacher{{50, 50, 0}},
    Bleacher{{100, 180, 0}},
};

SizedArray<GameEntity, 40> can;

#ifdef DEBUG
void exportToPLY(const char* filename,
                 int16_t potentialField[300 / SQUARE_SIZE_CM][200 / SQUARE_SIZE_CM], size_t width,
                 size_t height) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << '\n';
        return;
    }

    // Header for PLY file
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << (width * height) << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    // Write vertices (x, y, z)
    for (size_t x = 0; x < width; ++x) {
        for (size_t y = 0; y < height; ++y) {
            float z = potentialField[x][y];
            file << x << " " << y << " " << z << "\n";
        }
    }

    file.close();
    std::cout << "Exported potentialField to " << filename << '\n';
}

void visualizePotentialField(int16_t potentialField[300 / SQUARE_SIZE_CM][200 / SQUARE_SIZE_CM],
                             size_t width, size_t height) {
    const std::string gradient = " .,:;oO0@";

    int16_t minValue = potentialField[0][0];
    int16_t maxValue = potentialField[0][0];
    for (size_t x = 0; x < width; ++x) {
        for (size_t y = 0; y < height; ++y) {
            minValue = std::min(minValue, potentialField[x][y]);
            maxValue = std::max(maxValue, potentialField[x][y]);
        }
    }

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            float normalized =
                static_cast<float>(potentialField[x][y] - minValue) / (maxValue - minValue);
            int index = static_cast<int>(normalized * (gradient.size() - 1));
            std::cout << gradient[index] << " ";
        }
        std::cout << "\n";
    }
}
#endif

void thibault_top_init(config_t* config) {
    for (size_t x = 0; x < 300 / SQUARE_SIZE_CM; x++) {
        for (size_t y = 0; y < 200 / SQUARE_SIZE_CM; y++) {
            potential_field[x][y] = 0;
        }
    }

    for (auto& bleacher : bleachers) {
        auto& field = bleacher.potentialField();
        for (uint16_t x = 0; x < field.size(); x++) {
            for (uint16_t y = 0; y < field.size(); y++) {
                uint16_t xIndex = bleacher.x / SQUARE_SIZE_CM + x - field.size() / 2;
                uint16_t yIndex = bleacher.y / SQUARE_SIZE_CM + y - field[0].size() / 2;

                if (xIndex >= 300 / SQUARE_SIZE_CM || yIndex >= 200 / SQUARE_SIZE_CM) {
                    continue;
                }

                potential_field[xIndex][yIndex] += field[x][y];
            }
        }
    }

#ifdef DEBUG
    visualizePotentialField(potential_field, 300 / SQUARE_SIZE_CM, 200 / SQUARE_SIZE_CM);

    exportToPLY("/home/thibault/Documents/potentialField.ply", potential_field,
                300 / SQUARE_SIZE_CM, 200 / SQUARE_SIZE_CM);
#endif
}

void thibault_top_step(config_t* config, input_t* input, output_t* output, float x, float y) {
    int index_x = x / SQUARE_SIZE_CM;
    int index_y = y / SQUARE_SIZE_CM;

    if (index_x >= 300 / SQUARE_SIZE_CM || index_y >= 200 / SQUARE_SIZE_CM) {
        throw std::out_of_range("Coordinates out of range");
    }

    int potentials[8] = {
        potential_field[index_x - 1][index_y - 1], potential_field[index_x - 1][index_y],
        potential_field[index_x - 1][index_y + 1], potential_field[index_x][index_y - 1],
        potential_field[index_x][index_y + 1],     potential_field[index_x + 1][index_y - 1],
        potential_field[index_x + 1][index_y],     potential_field[index_x + 1][index_y + 1],
    };
    std::pair<int, int> best_potential = {0, potentials[0]};
    for (int i = 0; i < 8; i++) {
        if (potentials[i] < best_potential.second) {
            best_potential = {i, potentials[i]};
        }
    }

    std::printf("Best potential: %d\n", best_potential.first);

    switch (best_potential.first) {
        case 0:  // Top-left
            output->vitesse1_ratio = 0.3;
            output->vitesse2_ratio = 0.6;
            break;
        case 1:  // Top
            output->vitesse1_ratio = 0.5;
            output->vitesse2_ratio = 0.5;
            break;
        case 2:  // Top-right
            output->vitesse1_ratio = 0.6;
            output->vitesse2_ratio = 0.3;
            break;
        case 3:  // Left
            output->vitesse1_ratio = 0.4;
            output->vitesse2_ratio = 0.7;
            break;
        case 4:  // Right
            output->vitesse1_ratio = 0.7;
            output->vitesse2_ratio = 0.4;
            break;
        case 5:  // Bottom-left
            output->vitesse1_ratio = 0.6;
            output->vitesse2_ratio = 0.3;
            break;
        case 6:  // Bottom
            output->vitesse1_ratio = 0.5;
            output->vitesse2_ratio = 0.5;
            break;
        case 7:  // Bottom-right
            output->vitesse1_ratio = 0.3;
            output->vitesse2_ratio = 0.6;
            break;
        default:
            throw std::logic_error("Invalid potential index");
    }
}


#ifdef DEBUG
int main() {
    config_t config;
    input_t input;
    output_t output;

    thibault_top_init(&config);
    return 0;
}
#endif