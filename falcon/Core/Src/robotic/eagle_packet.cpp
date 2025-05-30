#include "robotic/eagle_packet.hpp"

#include <cmath>
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
    out.robot_detected = static_cast<bool>(br.get(1));
    out.robot_x_cm = br.get(9);
    out.robot_y_cm = br.get(8);
    out.robot_theta_deg = static_cast<int16_t>(br.get(9));

    out.opponent_detected = static_cast<bool>(br.get(1));
    out.opponent_x_cm = br.get(9);
    out.opponent_y_cm = br.get(8);
    out.opponent_theta_deg = static_cast<int16_t>(br.get(9));

    br.get(1);

    return true;
}
