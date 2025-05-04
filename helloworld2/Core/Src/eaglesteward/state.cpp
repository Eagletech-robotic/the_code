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

State::State() { init(); }

void State::init() {
    // Initialize the bleachers default positions, etc.
    world.reset();

    // Set the initial state for the IMU to field coordinate transformation.
    saveImuToFieldTransform(INITIAL_X, INITIAL_Y, INITIAL_ORIENTATION_DEGREES);
}

void State::print() const { myprintf("S %.2f %.2f  %.1f  %.3f\n", imu_x, imu_y, imu_theta_deg, filtered_tof_m); }

void State::startGame(uint32_t clock_ms) { start_time_ms = clock_ms; }

float State::elapsedTime(const input_t &input) const {
    return static_cast<float>(input.clock_ms - start_time_ms) / 1000.0f;
}

/**
 * Calculates the transformation from IMU coordinates to field coordinates and saves it in the state.
 */
void State::saveImuToFieldTransform(float x_field, float y_field, float theta_field) {
    // Calculate the rotation offset
    transformation_theta_deg = theta_field - imu_theta_deg;

    // Calculate the translation offsets
    float theta_offset_rad = transformation_theta_deg * (M_PI / 180.0f);
    transformation_x = x_field - (imu_x * cos(theta_offset_rad) - imu_y * sin(theta_offset_rad));
    transformation_y = y_field - (imu_x * sin(theta_offset_rad) + imu_y * cos(theta_offset_rad));
}

/**
 * Converts coordinates from IMU coordinate system to field coordinate system.
 * Returns the position (x, y) and orientation (theta) in the field coordinate system.
 */
void State::getPositionAndOrientation(float &out_x, float &out_y, float &out_theta) const {
    // Apply rotation and translation to convert coordinates
    float theta_offset_rad = transformation_theta_deg * (M_PI / 180.0f);
    out_x = imu_x * cos(theta_offset_rad) - imu_y * sin(theta_offset_rad) + transformation_x;
    out_y = imu_x * sin(theta_offset_rad) + imu_y * cos(theta_offset_rad) + transformation_y;

    // Convert angle from IMU system to field system
    out_theta = imu_theta_deg + transformation_theta_deg;
    out_theta = angle_normalize_deg(out_theta);
}

/**
 * Updates the state based on the input data: IMU, encoders, and TOF.
 */
void State::updateFromInput(const config_t &config, const input_t &input) {
    // Updates the robot's position and orientation based on the IMU and encoder data
    float delta_x_m, delta_y_m, delta_theta_deg;
    fusion_odo_imu_fuse(input.imu_accel_x_mss, input.imu_accel_y_mss, input.delta_yaw_deg, input.delta_encoder_left,
                        input.delta_encoder_right, config.time_step_s, imu_theta_deg, &delta_x_m, &delta_y_m,
                        &delta_theta_deg, 0.5f, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEELBASE_M);
    imu_x += delta_x_m;
    imu_y += delta_y_m;
    imu_theta_deg += delta_theta_deg;
    imu_theta_deg = angle_normalize_deg(imu_theta_deg);

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
    // Read the color
    color = eagle_packet.robot_colour == RobotColour::Blue ? Color::BLUE : Color::YELLOW;

    // Read our position and orientation, and calculate the transformation
    float x = static_cast<float>(eagle_packet.robot_x_cm) / 100.0f;
    float y = static_cast<float>(eagle_packet.robot_y_cm) / 100.0f;
    float theta_deg = eagle_packet.robot_orientation_deg;
    // Save the IMU -> field coordinate transformation in the state
    saveImuToFieldTransform(x, y, theta_deg);

    // Read the opponent's position and orientation
    opponent_x = static_cast<float>(eagle_packet.robot_x_cm) / 100.0f;
    opponent_y = static_cast<float>(eagle_packet.robot_y_cm) / 100.0f;
    opponent_theta_deg = eagle_packet.robot_orientation_deg;

    // Update the world from the packet
    world.reset_from_eagle_packet(eagle_packet);

    packet_received_at_this_step = true;
    myprintf("Packet received, state updated");
}
