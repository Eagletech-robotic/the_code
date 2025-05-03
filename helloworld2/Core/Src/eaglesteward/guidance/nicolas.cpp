#include "eaglesteward/guidance/nicolas.hpp"

#include <stdio.h>

#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/guidance/cc_root.hpp"
#include "eaglesteward/motor.hpp"
#include "eaglesteward/robot_constants.hpp"
#include "eaglesteward/state.hpp"
#include "eaglesteward/tof.hpp"
#include "robotic/angle.hpp"
#include "robotic/carre.hpp"
#include "robotic/command.hpp"
#include "robotic/fusion_odo_imu.hpp"
#include "utils/myprintf.hpp"

carre_t carre;
state_t nicolas_state;

void calcul_position(state_t *state, input_t *input, config_t *config) {
    // gestion de la position
    float delta_x_m = 0.0f;
    float delta_y_m = 0.0f;
    float delta_theta_deg = 0.0f;
    const float alpha_orientation_ratio = 0.5f;
    // O.O -> IMU seul
    fusion_odo_imu_fuse(input->imu_accel_x_mss, input->imu_accel_y_mss, input->delta_yaw_deg, input->delta_encoder_left,
                        input->delta_encoder_right, config->time_step_s, state->theta_deg, &delta_x_m, &delta_y_m,
                        &delta_theta_deg, alpha_orientation_ratio, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEELBASE_M);
    state->x_m += delta_x_m;
    state->y_m += delta_y_m;
    state->theta_deg += delta_theta_deg;
    // myprintf("delta_x=%.6f delta_y=%.6f delta_theta=%.3f\n", delta_x_m, delta_y_m, delta_theta_deg);
    state->theta_deg = angle_normalize_deg(state->theta_deg);
    print_state(state);
}

//  doit appeler la fonction et gérer les IOS
void nicolas_top_step(config_t *config, input_t *input, output_t *output) {
    nicolas_state.filtered_tof_m = tof_filter(nicolas_state, input->tof_m);
    // gestion de la position
    calcul_position(&nicolas_state, input, config);

    // gestion de la trajectoire
    // infinite_rectangle(config, input, output, &nicolas_state);
    Command command{};
    cc_root_behavior_tree(input, &command, &nicolas_state);

    // DEBUG
    if (!input->jack_removed) {
        command.target_left_speed = 0.0f;
        command.target_right_speed = 0.0f;
        if (input->blue_button) {
            command.shovel = ShovelCommand::SHOVEL_EXTEND;
        } else {
            command.shovel = ShovelCommand::SHOVEL_RETRACT;
        }
    }
    // END DEBUG

    // Send the command to the actuators (motors, shovel, LED)
    set_output(*config, *input, command, *output, nicolas_state);

    // myprintf("Ratios: left=%.3f, right=%.3f, pelle=%.3f\n", output->motor_left_ratio, output->motor_right_ratio,
    //          output->servo_pelle_ratio);
}

void nicolas_top_init(config_t *config) {
    config->time_step_s = 0.004f; // il faudrait 250hz, les get par I2C sont trop lent
    printf("cycle : %.0f ms\r\n", config->time_step_s * 1000.0);
    carre_init(&carre, config->time_step_s);
    motor_init(*config, nicolas_state);
    state_init(&nicolas_state);
}
