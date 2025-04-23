#include "eaglesteward/motor.hpp"
#include "robotic/robot_constants.hpp"

#include <cmath>

void motor_init(const config_t &config, state_t &state) {
    int const frequency = static_cast<int>(std::round(1.0f / config.time_step));

    pid_init(&state.pid_diff);
    // avec ki =0 est le seul moyen d'avoir R inférieur à 1% d'erreur (autour de 12-15v), ki fait diverger
    pid_tune(&state.pid_diff, 0.00033, 0.0000000, 0.000001);
    pid_limits(&state.pid_diff, -2.0, 2.0);
    pid_frequency(&state.pid_diff, frequency);

    pid_init(&state.pid_sum);
    // ki est toujours trop instable au démarrage.
    pid_tune(&state.pid_sum, 0.0003, 0.0000000, 0.000001);
    pid_limits(&state.pid_sum, -2.0, 2.0);
    pid_frequency(&state.pid_sum, frequency);
}

void motor_calculate_ratios(const config_t &config, state_t &state, const input_t &input, float speed_left_m_s,
                            float speed_right_m_s, float &out_motor_left_ratio, float &out_motor_right_ratio) {
    auto const sensor_left = static_cast<float>(input.encoder_left);
    auto const sensor_right = static_cast<float>(input.encoder_right);

    // 72000/(3.14*0.069)/250
    float const ticks_per_m = config.time_step * TICKS_PER_REV / WHEEL_CIRCUMFERENCE_M; //~1330
    float const ratio_left = speed_left_m_s * ticks_per_m;
    float const ratio_right = speed_right_m_s * ticks_per_m;

    float const regul_sum = pid_(&state.pid_sum, ratio_right + ratio_left, sensor_right + sensor_left);
    float const regul_diff = pid_(&state.pid_diff, ratio_right - ratio_left, sensor_right - sensor_left);

    out_motor_left_ratio = (regul_sum - regul_diff) / 2.0f;
    out_motor_right_ratio = (regul_sum + regul_diff) / 2.0f;
}
