#include "../platforms/shared/include/robotic/eagle_packet.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <sstream>
#include <stdexcept>
#include <vector>

/* ──────────────────────────────────────────────────────────────────────────
   Helper: encode an EaglePacket into the raw payload that sits between
   the starter byte and the checksum.  The layout exactly mirrors what the
   BitReader in eagle_packet.cpp expects (LSB‑first, 59‑bit header,
   no padding, each object = 16 bits).
   ──────────────────────────────────────────────────────────────────────── */
namespace {

class BitPacker {
  public:
    void push(uint32_t value, unsigned n_bits) {
        for (unsigned i = 0; i < n_bits; ++i) // LSB first
            bits_.push_back((value >> i) & 1u);
    }

    std::vector<uint8_t> to_bytes() const {
        std::vector<uint8_t> out((bits_.size() + 7) / 8, 0u);
        for (std::size_t i = 0; i < bits_.size(); ++i)
            if (bits_[i])
                out[i / 8] |= (1u << (i & 7));
        return out;
    }

  private:
    std::vector<uint8_t> bits_;
};

std::vector<uint8_t> build_payload(const EaglePacket &packet) {
    BitPacker bp;
    bp.push(static_cast<uint8_t>(packet.robot_colour), 1);
    bp.push(packet.robot_detected ? 1u : 0u, 1);
    bp.push(packet.robot_x_cm, 9);
    bp.push(packet.robot_y_cm, 8);
    bp.push(static_cast<uint16_t>(packet.robot_theta_deg), 9);

    bp.push(packet.opponent_detected ? 1u : 0u, 1);
    bp.push(packet.opponent_x_cm, 9);
    bp.push(packet.opponent_y_cm, 8);
    bp.push(static_cast<uint16_t>(packet.opponent_theta_deg), 9);

    bp.push(0u, 1);                  // padding bit 56

    return bp.to_bytes();
}

/* quick equality helper */
bool packets_equal(const EaglePacket &a, const EaglePacket &b) {
    if (std::memcmp(&a, &b, sizeof(EaglePacket)) == 0)
        return true;
    if (a.robot_colour != b.robot_colour)
        return false;
    if (a.robot_detected != b.robot_detected)
        return false;
    if (a.robot_x_cm != b.robot_x_cm)
        return false;
    if (a.robot_y_cm != b.robot_y_cm)
        return false;
    if (a.robot_theta_deg != b.robot_theta_deg)
        return false;
    if (a.opponent_detected != b.opponent_detected)
        return false;
    if (a.opponent_x_cm != b.opponent_x_cm)
        return false;
    if (a.opponent_y_cm != b.opponent_y_cm)
        return false;
    if (a.opponent_theta_deg != b.opponent_theta_deg)
        return false;
    return true;
}

} // namespace

/* ──────────────────────────────────────────────────────────────────────────
   TESTS
   ──────────────────────────────────────────────────────────────────────── */

TEST(EaglePacketDecode, RoundTripOneObject) {
    EaglePacket src{};
    src.robot_colour = RobotColour::Blue;
    src.robot_detected = true;
    src.robot_x_cm = 150;
    src.robot_y_cm = 100;
    src.robot_theta_deg = 45;

    src.opponent_detected = true;
    src.opponent_x_cm = 200;
    src.opponent_y_cm = 50;
    src.opponent_theta_deg = 90;

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}

TEST(EaglePacketDecode, BoundaryValues) {
    EaglePacket src{};
    src.robot_colour = RobotColour::Yellow;
    src.robot_detected = true;
    src.robot_x_cm = 300;
    src.robot_y_cm = 200;
    src.robot_theta_deg = 0;

    src.opponent_x_cm = 0;
    src.opponent_y_cm = 0;
    src.opponent_theta_deg = 359;
    src.opponent_detected = false;

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}

/* --------------------------------------------------------------------------
   Manual ground‑truth packet, hand‑built bit by bit to validate decoding.
   Header (9 bytes + 1 bit) and 1 object (20 bits) → 12‑byte payload.
   -------------------------------------------------------------------------- */
TEST(EaglePacketDecode, ManualBitPatternOneObject) {
    const std::vector<uint8_t> payload = {
        /* Byte 0  (bits 0‑7)
             bit0  robot_colour   =0 (blue)
             bit1  robot_detected =1
             bits2‑10 robot_x     =10  (bits2‑7 here) */
        0b00101010,

        /* Byte 1  (bits 8‑15)
             bits8‑10  robot_x (cont.)
             bits11‑18 robot_y  =20 (bits11‑15 here) */
        0b10100000,

        /* Byte 2  (bits 16‑23)
             bits16‑18 robot_y (cont.)
             bits19‑27 robot_theta =210 (bits19‑23 here) */
        0b10010000,

        /* Byte 3  (bits 24‑31)
             bits24‑27 robot_theta (cont.)
             bit28     opponent_detected =1
             bits29‑31 opponent_x =5 (bits29‑31 here) */
        0b10110110,

        /* Byte 4  (bits 32‑39)
             bits32‑37 opponent_x (cont.)
             bits38‑39 opponent_y =6 (bits38‑39 here) */
        0b10000000,

        /* Byte 5  (bits 40‑47)
             bits40‑45 opponent_y (cont.)
             bits46‑47 opponent_theta =90 (bits46‑47 here) */
        0b10000001,

        /* Byte 6  (bits 48‑55)
             bits48‑54 opponent_theta (cont.)
             bit55     padding =0 */
        0b00010110,
    };

    EaglePacket expected{};
    expected.robot_colour = RobotColour::Blue;
    expected.robot_detected = true;
    expected.robot_x_cm = 10;
    expected.robot_y_cm = 20;
    expected.robot_theta_deg = 210;

    expected.opponent_detected = true;
    expected.opponent_x_cm = 5;
    expected.opponent_y_cm = 6;
    expected.opponent_theta_deg = 90;

    EaglePacket out{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), out));

    if (!packets_equal(expected, out)) {
        std::ostringstream oss;
        oss << "Decoded packet:\n"
            << "robot_colour=" << static_cast<int>(out.robot_colour) << "\n"
            << "robot_detected=" << out.robot_detected << "\n"
            << "robot_x_cm=" << out.robot_x_cm << "\n"
            << "robot_y_cm=" << out.robot_y_cm << "\n"
            << "robot_theta_deg=" << out.robot_theta_deg << "\n"
            << "opponent_detected=" << out.opponent_detected << "\n"
            << "opponent_x_cm=" << out.opponent_x_cm << "\n"
            << "opponent_y_cm=" << out.opponent_y_cm << "\n"
            << "opponent_theta_deg=" << out.opponent_theta_deg << "\n";
        ADD_FAILURE() << oss.str();
    }

    EXPECT_TRUE(packets_equal(expected, out));
}
