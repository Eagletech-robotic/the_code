#include "eaglesteward/autopilot.hpp"
#include "eaglesteward/constante.hpp"

void autopilot_init(const config_t *config, state_t *state) {
    const float frequency = 1000.0 / config->time_step_ms;

    pid_init(&state->pid_diff);
    pid_tune(&state->pid_diff, 0.00033, 0.0000000,
             0.000001); // avec ki =0 est le seul moyen d'avoir R inférieur à 1% d'erreur
    // (autour de 12-15v), ki fait diverger
    pid_limits(&state->pid_diff, -2.0, 2.0);
    pid_frequency(&state->pid_diff, frequency);

    pid_init(&state->pid_sum);
    pid_tune(&state->pid_sum, 0.0003, 0.0000000, 0.000001); // ki est toujours trop instable au démarrage.
    pid_limits(&state->pid_sum, -2.0, 2.0);
    pid_frequency(&state->pid_sum, frequency);
}

// pour éviter de créer une nouvelle structure et pour évite de perdre le flux d'information, il faut envoyer un ouput
// vierge et recopier les données utiles. Cela évite de se perdre dans les mise à jours de valeurs
void autopilot_step(const config_t *config, state_t *state, const input_t *input, float v_gauche_m_s,
                    float v_droite_m_s, float *out_motor_left_ratio, float *out_motor_right_ratio) {
    const float sensor_left = input->encoder_left;
    const float sensor_right = input->encoder_right;

    // 72000/(3.14*0.069)/250
    const float ticks_per_m = (config->time_step_ms / 1000.0f) * TICKS_PER_REV / (WHEEL_CIRCUMFERENCE_M); //~1330
    const float ratio_left = v_gauche_m_s * ticks_per_m;
    const float ratio_right = v_droite_m_s * ticks_per_m;

    const float regul_sum = pid_(&state->pid_sum, ratio_right + ratio_left, sensor_right + sensor_left);
    const float regul_diff = pid_(&state->pid_diff, ratio_right - ratio_left, sensor_right - sensor_left);

    *out_motor_left_ratio = (regul_sum - regul_diff) / 2.0;
    *out_motor_right_ratio = (regul_sum + regul_diff) / 2.0;
}
