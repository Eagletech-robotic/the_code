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

    // for (bool &initial_bleacher : out.initial_bleachers) {
    //     initial_bleacher = static_cast<bool>(br.get(1));
    // }
    //
    // out.object_count = static_cast<uint8_t>(br.get(6));
    // if (out.object_count > 40)
    //     return false;
    //
    // br.get(1); // padding bit (71)
    //
    // for (uint8_t i = 0; i < out.object_count; ++i) {
    //     auto &object = out.objects[i];
    //     object.type = static_cast<ObjectType>(br.get(2));
    //
    //     auto const raw_x = static_cast<float>(br.get(8)); // 0‑255
    //     auto const raw_y = static_cast<float>(br.get(7)); // 0‑127
    //
    //     object.x_cm = static_cast<uint16_t>(std::round(raw_x * 300.0f / 255.0f)); // 0‑300 cm
    //     object.y_cm = static_cast<uint16_t>(std::round(raw_y * 200.0f / 127.0f)); // 0‑200 cm
    //     object.orientation_deg = static_cast<uint8_t>(br.get(3) * 30);
    // }
    return true;
}
