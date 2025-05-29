#include "iot01A/top_driver.h"
#include "eaglesteward/behavior.hpp"
#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/command.hpp"
#include "eaglesteward/motor.hpp"
#include "eaglesteward/state.hpp"
#include "utils/debug.hpp"
#include "utils/myprintf.hpp"

State top_state;

void top_init(config_t &config) {
    config.time_step_s = 0.004f;
    top_state = State(); // Call the constructor explicitly, as the simulator does not construct global variables.
    motor_init(config, top_state);
    behavior_init(&top_state);
}

void top_step(const config_t &config, const input_t &input, output_t &output) {
    // 1. Debug: print input
    // print_complete_input(input);

    // 2. Read the last Bluetooth packet (if available) and update the state
    auto const elapsed_time = top_state.elapsedTime(input);
    if(top_state.updateFromBluetooth(elapsed_time)) {
    	top_state.last_packet_time = elapsed_time;
    }

    // 3. Update position and orientation from IMU and encoders
    top_state.updateFromInput(config, input);

    // 4. Calculate the next command
    Command command{};
    top_behavior(&input, &command, &top_state);

    // 5. Perform some computations that fit in a step
    constexpr float MARGIN_TIME_FOR_LAST_RUN_US = 500.0f;
    float step_time_us = config.time_step_s * 1000.0f;
    top_state.world.do_some_calculations(
        [step_time_us]() { return timer_get_us() < step_time_us - MARGIN_TIME_FOR_LAST_RUN_US; });

    // 6. Convert the command to actuator commands (output)
    set_output(config, input, command, output, top_state);
    myprintf("T %.2f \n", timer_get_us() / 1000.0f);

    // 7. Debug: print output
    print_complete_output(output);
}
