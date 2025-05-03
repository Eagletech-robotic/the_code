#include "robotic/eagle_packet.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
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
    if (packet.object_count > 60)
        throw std::invalid_argument("object_count > 60");

    BitPacker bp;
    bp.push(static_cast<uint8_t>(packet.robot_colour), 1);
    bp.push(packet.robot_x_cm, 9);
    bp.push(packet.robot_y_cm, 8);
    bp.push(static_cast<uint16_t>(packet.robot_orientation_deg + 180), 9);

    bp.push(packet.opponent_x_cm, 9);
    bp.push(packet.opponent_y_cm, 8);
    bp.push(static_cast<uint16_t>(packet.opponent_orientation_deg + 180), 9);

    bp.push(packet.object_count, 6); // header ends at bit 58

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
    if (a.robot_x_cm != b.robot_x_cm)
        return false;
    if (a.robot_y_cm != b.robot_y_cm)
        return false;
    if (a.robot_orientation_deg != b.robot_orientation_deg)
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
    src.robot_x_cm = 150;
    src.robot_y_cm = 100;
    src.robot_orientation_deg = 45;

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
    src.robot_x_cm = 1;
    src.robot_y_cm = 2;
    src.robot_orientation_deg = 0;
    src.opponent_x_cm = 3;
    src.opponent_y_cm = 4;
    src.opponent_orientation_deg = 0;
    src.object_count = 0;

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}

TEST(EaglePacketDecode, MaxObjects) {
    EaglePacket src{};
    src.robot_colour = RobotColour::Yellow;
    for (uint8_t i = 0; i < 60; ++i) {
        src.objects[i] = {static_cast<ObjectType>(i % 3), uint16_t(i & 63), uint8_t(i % 32), uint8_t((i % 7) * 30)};
    }
    src.robot_orientation_deg = 30;
    src.opponent_orientation_deg = -30;
    src.object_count = 60;

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}

TEST(EaglePacketDecode, InvalidObjectCount) {
    /* object_count = 61 (binary 111101, LSB‑first)
       bits 53‑55 → bits 5‑7 of byte 6
       bits 56‑58 → bits 0‑2 of byte 7 */
    std::vector<uint8_t> payload(8, 0);
    payload[6] = 0b1010'0000; // bit 5 =1, bit 6 =0, bit 7 =1
    payload[7] = 0b0000'0111; // bits 0‑2 =111

    EaglePacket out{};
    EXPECT_FALSE(decode_eagle_packet(payload.data(), payload.size(), out));
}

TEST(EaglePacketDecode, TooShortHeader) {
    std::vector<uint8_t> tiny{0xAA, 0xBB, 0xCC}; // three bytes only
    EaglePacket out{};
    ASSERT_TRUE(decode_eagle_packet(tiny.data(), tiny.size(), out));
    EXPECT_EQ(out.object_count, 0); // zero‑padded under‑read
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
    EXPECT_EQ(out.objects[0].x_cm, 0); // zero‑padded
    EXPECT_EQ(out.objects[0].orientation_deg, 0);
}

TEST(EaglePacketDecode, BoundaryValues) {
    EaglePacket src{};
    src.robot_colour = RobotColour::Yellow;
    src.robot_x_cm = 300;
    src.robot_y_cm = 200;
    src.robot_orientation_deg = 180;

    src.opponent_x_cm = 0;
    src.opponent_y_cm = 0;
    src.opponent_orientation_deg = -180;

    src.object_count = 1;
    src.objects[0] = {ObjectType::Plank, 0, 0, 0};

    const auto payload = build_payload(src);

    EaglePacket dst{};
    ASSERT_TRUE(decode_eagle_packet(payload.data(), payload.size(), dst));
    EXPECT_TRUE(packets_equal(src, dst));
}
