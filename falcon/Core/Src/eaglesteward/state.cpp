#include "eaglesteward/state.hpp"

#include "eaglesteward/tof.hpp"
#include "robotic/bluetooth.hpp"
#include "robotic/constants.hpp"
#include "robotic/eagle_packet.hpp"
#include "robotic/fusion_odo_imu.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"

#include <math.h>

#include "utils/angles.hpp"

State::State() {}

void State::reset() {
    // Set the initial state for the IMU to field coordinate transformation.
    saveImuToFieldTransform(INITIAL_X, INITIAL_Y, INITIAL_ORIENTATION);
}

void State::print() const {
    const char *col;
    if (colour == RobotColour::Blue) {
        col = "BLUE";
    } else {
        col = "YELLOW";
    }
    float x, y, theta_deg;
    getPositionAndOrientation(x, y, theta_deg);
    myprintf("S %.2f %.2f %.1f TOF%.3f OPP%.2f %.2f %.1f %s\n", x, y, theta_deg, filtered_tof_m, opponent_x, opponent_y,
             to_degrees(opponent_theta), col);
}

bool State::hasGameStarted() const { return start_time_ms != -1; }

void State::gameNotStarted() { start_time_ms = -1; }

void State::startGame(const uint32_t clock_ms) { start_time_ms = static_cast<int32_t>(clock_ms); }

float State::elapsedTime(const input_t &input) const {
    if (start_time_ms == -1) {
        return 0.0f; // Game has not started yet
    } else {
        return static_cast<float>(input.clock_ms - start_time_ms) / 1000.0f;
    }
}

/**
 * Calculates the transformation from IMU coordinates to field coordinates and saves it in the state.
 */
void State::saveImuToFieldTransform(float x_field, float y_field, float theta_field) {
    // Calculate the rotation offset
    transformation_theta = theta_field - imu_theta;

    // Calculate the translation offsets
    transformation_x = x_field - (imu_x * cos(transformation_theta) - imu_y * sin(transformation_theta));
    transformation_y = y_field - (imu_x * sin(transformation_theta) + imu_y * cos(transformation_theta));
}

/**
 * Converts coordinates from IMU coordinate system to field coordinate system.
 * Returns the position (x, y) and orientation (theta) in the field coordinate system.
 */
void State::getPositionAndOrientation(float &out_x, float &out_y, float &out_theta) const {
    // Apply rotation and translation to convert coordinates
    out_x = imu_x * cos(transformation_theta) - imu_y * sin(transformation_theta) + transformation_x;
    out_y = imu_x * sin(transformation_theta) + imu_y * cos(transformation_theta) + transformation_y;

    // Convert angle from IMU system to field system
    out_theta = imu_theta + transformation_theta;
    out_theta = angle_normalize(out_theta);
}

/**
 * Updates the state based on the input data: IMU, encoders, and TOF.
 */
void State::updateFromInput(const config_t &config, const input_t &input) {
    // Updates the robot's position and orientation based on the IMU and encoder data
    float delta_x_m, delta_y_m, delta_theta;
    fusion_odo_imu_fuse(input.imu_accel_x_mss, input.imu_accel_y_mss, input.delta_yaw, input.delta_encoder_left,
                        input.delta_encoder_right, config.time_step_s, imu_theta, &delta_x_m, &delta_y_m, &delta_theta,
                        0.5f, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEELBASE_M);
    imu_x += delta_x_m;
    imu_y += delta_y_m;
    imu_theta = angle_normalize(imu_theta + delta_theta);

    // Filter the TOF value
    filtered_tof_m = tof_filter(*this, input.tof_m);

    print();
}

void State::updateFromBluetooth() {
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
    // Read the colour
    colour = eagle_packet.robot_colour;

    // Read our position and orientation, and calculate the transformation
    float x = static_cast<float>(eagle_packet.robot_x_cm) / 100.0f;
    float y = static_cast<float>(eagle_packet.robot_y_cm) / 100.0f;
    float theta = angle_normalize(to_radians(eagle_packet.robot_theta_deg));
    // Save the IMU -> field coordinate transformation in the state
    saveImuToFieldTransform(x, y, theta);

    // Read the opponent's position and orientation
    opponent_x = static_cast<float>(eagle_packet.opponent_x_cm) / 100.0f;
    opponent_y = static_cast<float>(eagle_packet.opponent_y_cm) / 100.0f;
    opponent_theta = angle_normalize(to_radians(eagle_packet.opponent_theta_deg));

    // Update the world from the packet
    world.update_from_eagle_packet(eagle_packet);

    packet_received_at_this_step = true;
    myprintf("Packet received, state updated");
}
