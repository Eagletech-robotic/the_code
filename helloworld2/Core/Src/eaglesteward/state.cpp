#include "eaglesteward/state.hpp"

#include "eaglesteward/robot_constants.hpp"
#include "eaglesteward/tof.hpp"
#include "robotic/angle.hpp"
#include "robotic/bluetooth.hpp"
#include "robotic/eagle_packet.hpp"
#include "robotic/fusion_odo_imu.hpp"
#include "utils/constants.hpp"
#include "utils/myprintf.hpp"

#include <math.h>

void print_state(const state_t &state) {
    myprintf("S %.2f %.2f  %.1f  %.3f\n", state.x_m, state.y_m, state.theta_deg, state.filtered_tof_m);
}

/**
 * Calculates the transformation from IMU coordinates to field coordinates and saves it in the state.
 */
void save_imu_to_field_transform(state_t &state, float x_field, float y_field, float theta_field) {
    // Calculate the rotation offset
    state.theta_offset_deg = theta_field - state.theta_deg;

    // Calculate the translation offsets
    float theta_offset_rad = state.theta_offset_deg * (M_PI / 180.0f);
    state.x_offset_m = x_field - (state.x_m * cos(theta_offset_rad) - state.y_m * sin(theta_offset_rad));
    state.y_offset_m = y_field - (state.x_m * sin(theta_offset_rad) + state.y_m * cos(theta_offset_rad));
}

void state_init(state_t &state) {
    state.x_m = .0f;
    state.y_m = .0f;
    state.theta_deg = .0f;
    state.target = 0;
    state.start_time_ms = 0;
    state.elapsed_time_s = .0f;
    state.filtered_tof_m = .0f;
    state.previous_jack_removed = false;

    // Set the initial state for the IMU to field coordinate transformation.
    save_imu_to_field_transform(state, INITIAL_X, INITIAL_Y, INITIAL_ORIENTATION_DEGREES);
}

/**
 * Converts coordinates from IMU coordinate system to field coordinate system.
 * Returns the position (x, y) and orientation (theta) in the field coordinate system.
 */
void get_position_and_orientation(const state_t &state, float &out_x, float &out_y, float &out_theta) {
    // Apply rotation and translation to convert coordinates
    float theta_offset_rad = state.theta_offset_deg * (M_PI / 180.0f);
    out_x = state.x_m * cos(theta_offset_rad) - state.y_m * sin(theta_offset_rad) + state.x_offset_m;
    out_y = state.x_m * sin(theta_offset_rad) + state.y_m * cos(theta_offset_rad) + state.y_offset_m;

    // Convert angle from IMU system to field system
    out_theta = state.theta_deg + state.theta_offset_deg;
    out_theta = angle_normalize_deg(out_theta);
}

/**
 * Updates the state based on the input data: IMU, encoders, and TOF.
 */
void update_state_from_input(const config_t &config, const input_t &input, state_t &state) {
    // Updates the robot's position and orientation based on the IMU and encoder data
    float delta_x_m, delta_y_m, delta_theta_deg;
    fusion_odo_imu_fuse(input.imu_accel_x_mss, input.imu_accel_y_mss, input.delta_yaw_deg, input.delta_encoder_left,
                        input.delta_encoder_right, config.time_step_s, state.theta_deg, &delta_x_m, &delta_y_m,
                        &delta_theta_deg, 0.5f, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEELBASE_M);
    state.x_m += delta_x_m;
    state.y_m += delta_y_m;
    state.theta_deg += delta_theta_deg;
    state.theta_deg = angle_normalize_deg(state.theta_deg);

    // Filter the TOF value
    state.filtered_tof_m = tof_filter(state, input.tof_m);

    print_state(state);
}

void update_state_from_bluetooth(state_t &state) {
    // Read until the last available packet
    const uint8_t *packet, *last_packet = nullptr;
    while ((packet = g_bluetooth_decoder.read_packet()) != nullptr)
        last_packet = packet;

    if (last_packet != nullptr) {
        // Decode the packet
        EaglePacket eagle_packet{};

        if (decode_eagle_packet(last_packet, PACKET_SIZE, eagle_packet)) {
            RobotColour robot_colour = eagle_packet.robot_colour;
            float x = static_cast<float>(eagle_packet.robot_x_cm) / 100.0f;
            float y = static_cast<float>(eagle_packet.robot_y_cm) / 100.0f;
            float theta_deg = eagle_packet.robot_orientation_deg;
            myprintf("Eagle packet: colour=%s, x=%.3f y=%.3f theta=%.3f\n",
                     robot_colour == RobotColour::Blue ? "B" : "Y", x, y, theta_deg);

            // Calculate the IMU -> field coordinate transformation and save it in the state
            save_imu_to_field_transform(state, x, y, theta_deg);
        }
    }
}
