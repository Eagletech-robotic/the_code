#include "robotic/eagle_packet.hpp"

#include <cstdint>

// --- tiny helper ----------------------------------------------------------
class BitReader {
  public:
    BitReader(const uint8_t *d, size_t bytes) : data_{d}, len_bits_{bytes * 8} {}

    uint32_t get(unsigned n) {
        uint32_t v = 0;
        for (unsigned i = 0; i < n; ++i) {
            if (bit_pos_ >= len_bits_)
                break; // zero‑pad overflow
            v |= ((data_[bit_pos_ >> 3] >> (bit_pos_ & 7)) & 1u) << i;
            ++bit_pos_;
        }
        return v;
    }

  private:
    const uint8_t *data_;
    size_t len_bits_;
    size_t bit_pos_{0};
};
// -------------------------------------------------------------------------

bool decode_eagle_packet(const uint8_t *payload, size_t payload_len, EaglePacket &out) {
    BitReader br{payload, payload_len};

    out.robot_colour = static_cast<RobotColour>(br.get(1));
    out.robot_x_cm = br.get(9);
    out.robot_y_cm = br.get(8);
    out.robot_orientation_deg = static_cast<int16_t>(br.get(9)) - 180;

    out.opponent_x_cm = br.get(9);
    out.opponent_y_cm = br.get(8);
    out.opponent_orientation_deg = static_cast<int16_t>(br.get(9)) - 180;

    out.object_count = static_cast<uint8_t>(br.get(6));
    if (out.object_count > 60)
        return false;

    for (uint8_t i = 0; i < out.object_count; ++i) {
        auto &o = out.objects[i];
        o.type = static_cast<ObjectType>(br.get(2));
        o.x_cm = br.get(6);
        o.y_cm = br.get(5);
        o.orientation_deg = static_cast<uint8_t>(br.get(3) * 30);
    }
    return true;
}
