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

    /* --------- discard packet if robot spun >5° in last 100 steps ------- */
    constexpr int ROT_CHECK_STEPS = 100; // 100 × 4 ms = 400 ms
    const float ROT_THRESH_RAD = to_radians(5.0f);

    float orient = 0.0f; // unwrapped orientation diff (rad)
    float min_o = 0.0f, max_o = 0.0f;
    int valid_steps = 0;

    for (int k = 0; k < ROT_CHECK_STEPS && k < RollingHistory::SIZE; ++k) {
        int idx = (odo_history.idx - 1 - k + RollingHistory::SIZE) % RollingHistory::SIZE;
        if (odo_history.dx[idx] == 0.0f && odo_history.dy[idx] == 0.0f && odo_history.dtheta[idx] == 0.0f)
            break; // history not yet primed

        orient -= odo_history.dtheta[idx]; // accumulate backwards
        min_o = std::min(min_o, orient);
        max_o = std::max(max_o, orient);
        ++valid_steps;
    }

    const float rot_span = max_o - min_o; // always ≥ 0

    if (rot_span > ROT_THRESH_RAD && eagle_packet.robot_detected) {
        printf("BT ignored rot span %.2f\n", rot_span * 180.0f / M_PI);
        /* we still use the packet for opponent & world updates below */
        eagle_packet.robot_detected = false; // suppress self-pose use
    }

    if (eagle_packet.robot_detected) {
        constexpr int MAX_AGE = 100; // nb steps

        const float odom_x = robot_x;
        const float odom_y = robot_y;
        const float odom_theta = robot_theta;

        /* 1. Camera pose (at capture time) */
        const float cam_x = static_cast<float>(eagle_packet.robot_x_cm) * 0.01f;
        const float cam_y = static_cast<float>(eagle_packet.robot_y_cm) * 0.01f;
        const float cam_theta = angle_normalize(to_radians(eagle_packet.robot_theta_deg));

        /* 2. Walk back in odometry history to find pose nearest to camera */
        float test_x = robot_x, test_y = robot_y, test_theta = robot_theta;
        float best_x = test_x, best_y = test_y, best_theta = test_theta;
        float min_d2 = FLT_MAX;
        int best_k = 0;

        for (int k = 0; k < MAX_AGE; ++k) {
            float dx = test_x - cam_x;
            float dy = test_y - cam_y;
            float d2 = dx * dx + dy * dy;

            if (d2 < min_d2) {
                min_d2 = d2;
                best_k = k;
                best_x = test_x;
                best_y = test_y;
                best_theta = test_theta;
            }

            int idx_back = (odo_history.idx - 1 - k + RollingHistory::SIZE) % RollingHistory::SIZE;
            if (odo_history.dx[idx_back] == 0.0f && odo_history.dy[idx_back] == 0.0f &&
                odo_history.dtheta[idx_back] == 0.0f)
                break; // history not yet filled

            test_x -= odo_history.dx[idx_back];
            test_y -= odo_history.dy[idx_back];
            test_theta = angle_normalize(test_theta - odo_history.dtheta[idx_back]);
        }

        /* 3. Orientation error between best historic pose and camera pose */
        const float dtheta_err = angle_normalize(cam_theta - best_theta);
        constexpr int STEP_MS = 4;
        const int32_t latency_ms = best_k * STEP_MS;
        printf("BT latency %d ms dx=%.3f dy=%.3f dth=%.3f\n", latency_ms, cam_x - best_x, cam_y - best_y,
               to_degrees(dtheta_err));

        const float cos_err = std::cos(dtheta_err);
        const float sin_err = std::sin(dtheta_err);

        /* 4. Re-integrate deltas from capture time up to now,
              rotating each delta by orientation error                     */
        float corr_x = cam_x;
        float corr_y = cam_y;
        float corr_theta = cam_theta;

        for (int k = best_k; k > 0; --k) {
            int idx_fwd = (odo_history.idx - k + RollingHistory::SIZE) % RollingHistory::SIZE;
            float dx = odo_history.dx[idx_fwd];
            float dy = odo_history.dy[idx_fwd];

            /* rotate delta to camera frame */
            float rdx = cos_err * dx - sin_err * dy;
            float rdy = sin_err * dx + cos_err * dy;

            corr_x += rdx;
            corr_y += rdy;
            corr_theta = angle_normalize(corr_theta + odo_history.dtheta[idx_fwd]);
        }

        /* 5. Complementary-filter blend with current odometry */
        constexpr float ALPHA = 1.0f; // tune 0-1
        robot_x = odom_x * (1.0f - ALPHA) + corr_x * ALPHA;
        robot_y = odom_y * (1.0f - ALPHA) + corr_y * ALPHA;
        float dtheta_blend = angle_normalize(corr_theta - odom_theta);
        robot_theta = angle_normalize(odom_theta + ALPHA * dtheta_blend);
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
