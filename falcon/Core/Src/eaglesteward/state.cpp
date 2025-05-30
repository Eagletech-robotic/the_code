#include "eaglesteward/state.hpp"

#include <cfloat>

#include "eaglesteward/constants.hpp"
#include "eaglesteward/tof.hpp"
#include "robotic/bluetooth.hpp"
#include "robotic/eagle_packet.hpp"
#include "robotic/fusion_odo_imu.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"

void State::reset() {
    robot_x = INITIAL_X;
    robot_y = INITIAL_Y;
    robot_theta = INITIAL_ORIENTATION;
    odo_history.clear();
}

void State::print() const {
    const char *col;
    if (colour == RobotColour::Blue) {
        col = "BLU";
    } else {
        col = "YLW";
    }
    myprintf("ROB %s %.3f,%.3f,%.0f TOF %.3f OPP %.3f,%.3f,%.0f\n", col, robot_x, robot_y, to_degrees(robot_theta),
             filtered_tof_m, world.opponent_x, world.opponent_y, to_degrees(world.opponent_theta));
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
 * Updates the state based on the input data: IMU, encoders, and TOF.
 */
void State::updateFromInput(const config_t &config, const input_t &input) {
    // Updates the robot's position and orientation based on the IMU and encoder data
    float delta_x_m, delta_y_m, delta_theta;
    fusion_odo_imu_fuse(input.imu_accel_x_mss, input.imu_accel_y_mss, input.delta_yaw, input.delta_encoder_left,
                        input.delta_encoder_right, config.time_step_s, robot_theta, &delta_x_m, &delta_y_m,
                        &delta_theta, 0.5f, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEELBASE_M);
    robot_x += delta_x_m;
    robot_y += delta_y_m;
    robot_theta = angle_normalize(robot_theta + delta_theta);

    // Memorise the odometry history
    odo_history.push(delta_x_m, delta_y_m, delta_theta);

    // Filter the TOF value
    filtered_tof_m = tof_filter(*this, input.tof_m);

    print();
}

void State::updateFromBluetooth(float elapsed_time) {
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

    // Read our robot position
    constexpr int MIN_TIME_BETWEEN_ROBOT_UPDATES = 1.0f; // seconds
    if (eagle_packet.robot_detected && (elapsed_time - last_robot_update_time) >= MIN_TIME_BETWEEN_ROBOT_UPDATES) {
        float const camera_x = static_cast<float>(eagle_packet.robot_x_cm) * 0.01f;
        float const camera_y = static_cast<float>(eagle_packet.robot_y_cm) * 0.01f;
        float const camera_theta = angle_normalize(to_radians(eagle_packet.robot_theta_deg));

        float corrected_x, corrected_y, corrected_theta;
        odo_history.rectify_odometry_from_camera_position(camera_x, camera_y, camera_theta, robot_x, robot_y,
                                                          robot_theta, corrected_x, corrected_y, corrected_theta);

        // Blend with current odometry
        constexpr float CAMERA_GAIN_POSITION = 0.8f; // the closest tp 1, the more we trust the camera
        constexpr float CAMERA_GAIN_THETA = 0.8f;

        robot_x = robot_x * (1.0f - CAMERA_GAIN_POSITION) + corrected_x * CAMERA_GAIN_POSITION;
        robot_y = robot_y * (1.0f - CAMERA_GAIN_POSITION) + corrected_y * CAMERA_GAIN_POSITION;
        robot_theta = angle_normalize(robot_theta + CAMERA_GAIN_THETA * angle_normalize(corrected_theta - robot_theta));
        last_robot_update_time = elapsed_time;
    }

    // Read the World properties from the packet
    world.update_from_eagle_packet(eagle_packet, elapsed_time);

    packet_received_at_this_step = true;
    myprintf("Packet received, state updated");
}
