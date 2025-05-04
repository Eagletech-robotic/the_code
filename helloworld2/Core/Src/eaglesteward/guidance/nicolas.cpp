#include "eaglesteward/guidance/nicolas.hpp"

#include <stdio.h>

#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/guidance/cc_root.hpp"
#include "eaglesteward/motor.hpp"
#include "eaglesteward/state.hpp"
#include "robotic/carre.hpp"
#include "robotic/command.hpp"
#include "utils/myprintf.hpp"

carre_t carre;
State nicolas_state;

void nicolas_top_init(config_t &config) {
    config.time_step_s = 0.004f; // il faudrait 250hz, les get par I2C sont trop lent
    nicolas_state.init();
    motor_init(config, nicolas_state);
    // carre_init(&carre, config.time_step_s);
}

void nicolas_top_step(const config_t &config, const input_t &input, output_t &output) {
    // 1. Debug: print input
    // print_complete_input(input);

    // 2. Update position and orientation from IMU and encoders
    nicolas_state.updateFromInput(config, input);

    // 3. Read the last Bluetooth packet (if available) and update the state
    nicolas_state.updateFromBluetooth();

    // 4. Calculate the next command
    Command command{};
    // cc_infinite_rectangle(&input, &command, &nicolas_state);
    cc_root_behavior_tree(&input, &command, &nicolas_state);

    // DEBUG -> to be moved to the behavior tree
    if (!input.jack_removed) {
        command.target_left_speed = 0.0f;
        command.target_right_speed = 0.0f;
        if (input.blue_button) {
            command.shovel = ShovelCommand::SHOVEL_EXTENDED;
        } else {
            command.shovel = ShovelCommand::SHOVEL_RETRACTED;
        }
    }
    // END DEBUG

    // 5. Convert the command to actuator commands (output)
    set_output(config, input, command, output, nicolas_state);

    // 6. Debug: print output
    // print_complete_output(output);
    // myprintf("Ratios: left=%.3f, right=%.3f, pelle=%.3f\n", output.motor_left_ratio, output.motor_right_ratio,
    //          output.shovel_ratio);
}
