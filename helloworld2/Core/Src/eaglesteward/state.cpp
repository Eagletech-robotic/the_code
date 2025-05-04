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
    myprintf("S %.2f %.2f  %.1f  %.3f\n", state.imu_x, state.imu_y, state.imu_theta_deg, state.filtered_tof_m);
}

/**
 * Calculates the transformation from IMU coordinates to field coordinates and saves it in the state.
 */
void save_imu_to_field_transform(state_t &state, float x_field, float y_field, float theta_field) {
    // Calculate the rotation offset
    state.transformation_theta_deg = theta_field - state.imu_theta_deg;

    // Calculate the translation offsets
    float theta_offset_rad = state.transformation_theta_deg * (M_PI / 180.0f);
    state.transformation_x = x_field - (state.imu_x * cos(theta_offset_rad) - state.imu_y * sin(theta_offset_rad));
    state.transformation_y = y_field - (state.imu_x * sin(theta_offset_rad) + state.imu_y * cos(theta_offset_rad));
}

void state_init(state_t &state) {
    // Set the initial state for the IMU to field coordinate transformation.
    save_imu_to_field_transform(state, INITIAL_X, INITIAL_Y, INITIAL_ORIENTATION_DEGREES);
}

/**
 * Converts coordinates from IMU coordinate system to field coordinate system.
 * Returns the position (x, y) and orientation (theta) in the field coordinate system.
 */
void get_position_and_orientation(const state_t &state, float &out_x, float &out_y, float &out_theta) {
    // Apply rotation and translation to convert coordinates
    float theta_offset_rad = state.transformation_theta_deg * (M_PI / 180.0f);
    out_x = state.imu_x * cos(theta_offset_rad) - state.imu_y * sin(theta_offset_rad) + state.transformation_x;
    out_y = state.imu_x * sin(theta_offset_rad) + state.imu_y * cos(theta_offset_rad) + state.transformation_y;

    // Convert angle from IMU system to field system
    out_theta = state.imu_theta_deg + state.transformation_theta_deg;
    out_theta = angle_normalize_deg(out_theta);
}

/**
 * Updates the state based on the input data: IMU, encoders, and TOF.
 */
void update_state_from_input(const config_t &config, const input_t &input, state_t &state) {
    // Updates the robot's position and orientation based on the IMU and encoder data
    float delta_x_m, delta_y_m, delta_theta_deg;
    fusion_odo_imu_fuse(input.imu_accel_x_mss, input.imu_accel_y_mss, input.delta_yaw_deg, input.delta_encoder_left,
                        input.delta_encoder_right, config.time_step_s, state.imu_theta_deg, &delta_x_m, &delta_y_m,
                        &delta_theta_deg, 0.5f, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEELBASE_M);
    state.imu_x += delta_x_m;
    state.imu_y += delta_y_m;
    state.imu_theta_deg += delta_theta_deg;
    state.imu_theta_deg = angle_normalize_deg(state.imu_theta_deg);

    // Filter the TOF value
    state.filtered_tof_m = tof_filter(state, input.tof_m);

    print_state(state);
}

void update_state_from_bluetooth(state_t &state) {
    // Read until the last available packet
    const uint8_t *packet, *last_packet = nullptr;
    while ((packet = g_bluetooth_decoder.read_packet()) != nullptr)
        last_packet = packet;

    // Stop here if there is no new packet
    if (last_packet == nullptr)
        return;

    // Decode the packet. Stop here if it fails.
    EaglePacket eagle_packet{};
    if (!decode_eagle_packet(last_packet, PACKET_SIZE, eagle_packet)) {
        myprintf("decode_eagle_packet() failed\n");
        return;
    }

    // ------ DECODING -------
    // Read the color
    state.color = eagle_packet.robot_colour == RobotColour::Blue ? Color::BLUE : Color::YELLOW;

    // Read our position and orientation, and calculate the transformation
    float x = static_cast<float>(eagle_packet.robot_x_cm) / 100.0f;
    float y = static_cast<float>(eagle_packet.robot_y_cm) / 100.0f;
    float theta_deg = eagle_packet.robot_orientation_deg;
    // Save the IMU -> field coordinate transformation in the state
    save_imu_to_field_transform(state, x, y, theta_deg);

    // Read the opponent's position and orientation
    state.opponent_x = static_cast<float>(eagle_packet.robot_x_cm) / 100.0f;
    state.opponent_y = static_cast<float>(eagle_packet.robot_y_cm) / 100.0f;
    state.opponent_theta_deg = eagle_packet.robot_orientation_deg;

    // Update the world from the packet
    state.world.reset_from_eagle_packet(eagle_packet);

    myprintf("Packet received, state updated");
}
