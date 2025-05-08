#include "robotic/eagle_packet.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>
#include <sstream>

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
    if (packet.object_count > 60)
        throw std::invalid_argument("object_count > 60");

    BitPacker bp;
    bp.push(static_cast<uint8_t>(packet.robot_colour), 1);
    bp.push(packet.robot_detected ? 1u : 0u, 1);
    bp.push(packet.robot_x_cm, 9);
    bp.push(packet.robot_y_cm, 8);
    bp.push(static_cast<uint16_t>(packet.robot_orientation_deg + 180), 9);

    bp.push(packet.opponent_detected ? 1u : 0u, 1);
    bp.push(packet.opponent_x_cm, 9);
    bp.push(packet.opponent_y_cm, 8);
    bp.push(static_cast<uint16_t>(packet.opponent_orientation_deg + 180), 9);

    bp.push(packet.object_count, 6); // object_count bits 55-60
    bp.push(0u, 3);                  // padding bits 61-63

    for (uint8_t i = 0; i < packet.object_count; ++i) {
        const auto &object = packet.objects[i];
        bp.push(static_cast<uint8_t>(object.type), 2);

        const auto raw_x = static_cast<uint8_t>(std::round(static_cast<float>(object.x_cm) * 63.0f / 300.0f));
        const auto raw_y = static_cast<uint8_t>(std::round(static_cast<float>(object.y_cm) * 31.0f / 200.0f));

        bp.push(raw_x, 6); // 0‑63
        bp.push(raw_y, 5); // 0‑31
        bp.push(object.orientation_deg / 30, 3);
    }
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
    if (a.robot_orientation_deg != b.robot_orientation_deg)
        return false;
    if (a.opponent_detected != b.opponent_detected)
        return false;
    if (a.opponent_x_cm != b.opponent_x_cm)
        return false;
    if (a.opponent_y_cm != b.opponent_y_cm)
        return false;
    if (a.opponent_orientation_deg != b.opponent_orientation_deg)
        return false;
    if (a.object_count != b.object_count)
        return false;

    for (uint8_t i = 0; i < a.object_count; ++i) {
        const auto cm_tol = 3; // ≤3 cm difference is OK after quantisation
        if (a.objects[i].type != b.objects[i].type)
            return false;
        if (std::abs(int(a.objects[i].x_cm) - int(b.objects[i].x_cm)) > cm_tol)
            return false;
        if (std::abs(int(a.objects[i].y_cm) - int(b.objects[i].y_cm)) > cm_tol)
            return false;
        if (a.objects[i].orientation_deg != b.objects[i].orientation_deg)
            return false;
    }
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
    src.robot_orientation_deg = 45;

    src.opponent_detected = true;
    src.opponent_x_cm = 200;
    src.opponent_y_cm = 50;
    src.opponent_orientation_deg = -90;

    src.object_count = 1;
    src.objects[0] = {ObjectType::Bleacher, 25, 15, 60};

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}

TEST(EaglePacketDecode, ZeroObjects) {
    EaglePacket src{};
    src.robot_colour = RobotColour::Blue;
    src.robot_detected = false;
    src.robot_x_cm = 1;
    src.robot_y_cm = 2;
    src.robot_orientation_deg = 0;
    src.opponent_x_cm = 3;
    src.opponent_y_cm = 4;
    src.opponent_orientation_deg = 0;
    src.opponent_detected = false;
    src.object_count = 0;

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}

TEST(EaglePacketDecode, MaxObjects) {
    EaglePacket src{};
    src.robot_colour = RobotColour::Yellow;
    src.robot_detected = true;
    for (uint8_t i = 0; i < 60; ++i) {
        src.objects[i] = {static_cast<ObjectType>(i % 3), uint16_t(i & 63), uint8_t(i % 32), uint8_t((i % 7) * 30)};
    }
    src.robot_orientation_deg = 30;
    src.opponent_detected = true;
    src.opponent_orientation_deg = -30;
    src.object_count = 60;

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}

TEST(EaglePacketDecode, InvalidObjectCount) {
    /* object_count = 61 (binary 111101, LSB-first)
       bit 55  → bit 7 of byte 6
       bits 56-60 → bits 0-4 of byte 7 */
    std::vector<uint8_t> payload(8, 0);
    payload[6] = 0b1000'0000;            // bit 7 = 1 (LSB of 111101)
    payload[7] = 0b0001'1110;            // bits 0-4 = 0 1 1 1 1 (rest of 111101)

    EaglePacket out{};
    EXPECT_FALSE(decode_eagle_packet(payload.data(), payload.size(), out));
}

TEST(EaglePacketDecode, TooShortHeader) {
    std::vector<uint8_t> tiny{0xAA, 0xBB, 0xCC}; // three bytes only
    EaglePacket out{};
    ASSERT_TRUE(decode_eagle_packet(tiny.data(), tiny.size(), out));
    EXPECT_EQ(out.object_count, 0); // zero-padded under-read
}

TEST(EaglePacketDecode, TooShortForObjects) {
    /* declare 3 objects but supply none */
    EaglePacket hdr_only{};
    hdr_only.object_count = 3;
    const auto payload = build_payload(hdr_only);
    std::vector<uint8_t> truncated(payload.begin(), payload.begin() + 8); // keep header only

    EaglePacket out{};
    ASSERT_TRUE(decode_eagle_packet(truncated.data(), truncated.size(), out));
    EXPECT_EQ(out.object_count, 3);
    EXPECT_EQ(out.objects[0].x_cm, 0); // zero-padded
    EXPECT_EQ(out.objects[0].orientation_deg, 0);
}

TEST(EaglePacketDecode, BoundaryValues) {
    EaglePacket src{};
    src.robot_colour = RobotColour::Yellow;
    src.robot_detected = true;
    src.robot_x_cm = 300;
    src.robot_y_cm = 200;
    src.robot_orientation_deg = 180;

    src.opponent_x_cm = 0;
    src.opponent_y_cm = 0;
    src.opponent_orientation_deg = -180;
    src.opponent_detected = false;

    src.object_count = 1;
    src.objects[0] = {ObjectType::Plank, 0, 0, 0};

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}

TEST(EaglePacketDecode, ManualBitPatternOneObject) {
    /* Packet built manually bit-by-bit (see comment inside the test).
       Header (8 bytes) and one object (2 bytes) → 10-byte payload. */

    const std::vector<uint8_t> payload = {
        /* Byte 0  (bits 0-7): 0b00101010
             bit0 robot_colour   =0 (blue)
             bit1 robot_detected =1
             bits2-10 robot_x    =10 (LSB first) [bits2-7 here]
        */ 0b00101010,

        /* Byte 1  (bits 8-15): 0b10100000
             bits8-10 robot_x (cont.)
             bits11-18 robot_y =20  (bits11-15 here)
        */ 0b10100000,

        /* Byte 2  (bits 16-23): 0b10010000
             bits16-18 robot_y (cont.)
             bits19-27 robot_orientation=210 (bits19-23 here)
        */ 0b10010000,

        /* Byte 3  (bits 24-31): 0b10100110
             bits24-27 robot_orientation (cont.)
             bit28 opponent_detected =1
             bits29-31 opponent_x =5 (bits29-31 here) */
        0b10110110,

        /* Byte 4  (bits 32-39): 0b10000000
             bits32-37 opponent_x (cont.)
             bits38-39 opponent_y =6 (bits38-39 here) */
        0b10000000,

        /* Byte 5  (bits 40-47): 0b10000001
             bits40-45 opponent_y (cont.)
             bits46-47 opponent_orientation =90 (bits46-47 here) */
        0b10000001,

        /* Byte 6  (bits 48-55): 0b10010110
             bits48-54 opponent_orientation (cont.)
             bit55 object_count bit0 =1 */
        0b10010110,

        /* Byte 7  (bits 56-63): 0b00000000
             bits56-60 remaining object_count bits (all 0)
             bits61-63 padding =0 */
        0b00000000,

        /* Byte 8  (bits 64-71): 0b00001100
             object[0] type=0 (bits64-65)
             raw_x=3 (bits66-71) */
        0b00001100,

        /* Byte 9  (bits 72-79): 0b01000101
             raw_y=5 (bits72-76)
             θ index=2 (bits77-79) */
        0b01000101
    };

    EaglePacket expected{};
    expected.robot_colour = RobotColour::Blue;
    expected.robot_detected = true;
    expected.robot_x_cm = 10;
    expected.robot_y_cm = 20;
    expected.robot_orientation_deg = 30;

    expected.opponent_detected = true;
    expected.opponent_x_cm = 5;
    expected.opponent_y_cm = 6;
    expected.opponent_orientation_deg = -90;

    expected.object_count = 1;
    expected.objects[0] = {ObjectType::Bleacher, 14, 32, 60};

    EaglePacket out{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), out));

    if (!packets_equal(expected, out)) {
        std::ostringstream oss;
        oss << "Decoded packet:\n"
            << "robot_colour=" << static_cast<int>(out.robot_colour) << "\n"
            << "robot_detected=" << out.robot_detected << "\n"
            << "robot_x_cm=" << out.robot_x_cm << "\n"
            << "robot_y_cm=" << out.robot_y_cm << "\n"
            << "robot_orientation_deg=" << out.robot_orientation_deg << "\n"
            << "opponent_detected=" << out.opponent_detected << "\n"
            << "opponent_x_cm=" << out.opponent_x_cm << "\n"
            << "opponent_y_cm=" << out.opponent_y_cm << "\n"
            << "opponent_orientation_deg=" << out.opponent_orientation_deg << "\n"
            << "object_count=" << static_cast<int>(out.object_count) << "\n"
            << "object[0].type=" << static_cast<int>(out.objects[0].type) << "\n"
            << "object[0].x_cm=" << out.objects[0].x_cm << "\n"
            << "object[0].y_cm=" << out.objects[0].y_cm << "\n"
            << "object[0].orientation_deg=" << out.objects[0].orientation_deg << std::endl;
        ADD_FAILURE() << oss.str();
    }

    EXPECT_TRUE(packets_equal(expected, out));
}
