#include "eaglesteward/state.hpp"

#include <cfloat>

#include "eaglesteward/constants.hpp"
#include "eaglesteward/tof.hpp"
#include "robotic/bluetooth.hpp"
#include "robotic/eagle_packet.hpp"
#include "robotic/fusion_odo_imu.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"

#include <math.h>

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

    if (eagle_packet.robot_detected) {
        float const camera_x = static_cast<float>(eagle_packet.robot_x_cm) * 0.01f;
        float const camera_y = static_cast<float>(eagle_packet.robot_y_cm) * 0.01f;
        float const camera_theta = angle_normalize(to_radians(eagle_packet.robot_theta_deg));

        // Walk back in odometry history to find pose nearest to camera
        int lookback_steps = 0;
        odo_history.find_nearest_pose(camera_x - robot_x, camera_y - robot_y, RollingHistory::SIZE, lookback_steps);
        // myprintf("find_nearest_pose(%.3f %.3f) => %d\n", camera_x - robot_x, camera_y - robot_y, lookback_steps);
        // odo_history.print_for_debug();

        // Calculate the relative path since the camera pose
        float relative_x, relative_y, relative_theta;
        odo_history.integrate_last_steps(lookback_steps, relative_x, relative_y, relative_theta);

        // Calculate the amplitude of our rotations around the camera pose
        float rotation_span;
        odo_history.rotation_span_in_area(-relative_x - 0.05f, -relative_y - 0.05f, -relative_x + 0.05f,
                                          -relative_y + 0.05f, RollingHistory::SIZE, rotation_span);

        // Only trust the camera orientation for limited rotation amplitudes
        float corrected_theta = robot_theta;
        if (rotation_span <= to_radians(2.0f)) {
            corrected_theta = angle_normalize(camera_theta + relative_theta);
        } else {
            printf("BT TH-IGN span=%.3f)\n", to_degrees(rotation_span));
        }
        float const error_theta = angle_normalize(corrected_theta - robot_theta);

        // Re-integrate deltas from capture time up to now, rotating the cumulative trajectory by orientation error
        float const cos_err = std::cos(error_theta);
        float const sin_err = std::sin(error_theta);
        float const corrected_x = camera_x + cos_err * relative_x - sin_err * relative_y;
        float const corrected_y = camera_y + sin_err * relative_x + cos_err * relative_y;

        // Optionally log the error
        float const error_x = corrected_x - robot_x;
        float const error_y = corrected_y - robot_y;
        printf("BT %d ms %.3f %.3f %.3f\n", lookback_steps * 4, error_x, error_y, to_degrees(error_theta));

        // Blend with current odometry
        constexpr float CAMERA_GAIN = 0.5f; // the closest tp 1, the more we trust the camera
        robot_x = robot_x * (1.0f - CAMERA_GAIN) + corrected_x * CAMERA_GAIN;
        robot_y = robot_y * (1.0f - CAMERA_GAIN) + corrected_y * CAMERA_GAIN;
        robot_theta = angle_normalize(robot_theta + CAMERA_GAIN * angle_normalize(corrected_theta - robot_theta));
    }

    if (eagle_packet.opponent_detected) {
        // Read the opponent's position and orientation
        world.opponent_x = static_cast<float>(eagle_packet.opponent_x_cm) / 100.0f;
        world.opponent_y = static_cast<float>(eagle_packet.opponent_y_cm) / 100.0f;
        world.opponent_theta = angle_normalize(to_radians(eagle_packet.opponent_theta_deg));
    }

    // Update the world from the packet
    world.update_from_eagle_packet(eagle_packet);

    packet_received_at_this_step = true;
    myprintf("Packet received, state updated");
}
