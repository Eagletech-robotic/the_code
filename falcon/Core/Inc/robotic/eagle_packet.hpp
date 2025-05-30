#pragma once
#include <cstddef>
#include <cstdint>

/* --------------------------------------------------------------------------
   Bluetooth packet format (little‑endian bit order).
   Packet length: 7 bytes

    Bit‑index  Purpose                                Size (bits)
   ─────────────────────────────────────────────────────────────────
   0          robot_colour (0=blue, 1=yellow)        1
   1          robot_detected (0=no, 1=yes)           1
   2‑10       robot_x (0–300)                        9
   11‑18      robot_y (0–200)                        8
   19‑27      robot_theta_deg [0, 360)               9
   28         opponent_detected (0=no, 1=yes)        1
   29‑37      opponent_x (0–300)                     9
   38‑45      opponent_y (0–200)                     8
   46‑54      opponent_theta_deg [0, 360)            9
   55         padding (0)                            1
   -------------------------------------------------------------------------- */

enum class RobotColour : uint8_t { Blue = 0, Yellow = 1 };

enum class ObjectType : uint8_t { Bleacher = 0, Plank = 1, Can = 2 };

struct EagleObject {
    ObjectType type;
    uint16_t x_cm;
    uint16_t y_cm;
    uint16_t orientation_deg;
};

struct EaglePacket {
    RobotColour robot_colour;
    bool robot_detected;
    uint16_t robot_x_cm;
    uint16_t robot_y_cm;
    int16_t robot_theta_deg;

    bool opponent_detected;
    uint16_t opponent_x_cm;
    uint16_t opponent_y_cm;
    int16_t opponent_theta_deg;
};

// Decode the payload sitting between starter byte and checksum.
// Returns true on success; false if object_count > 40 or payload too short.
bool decode_eagle_packet(const uint8_t *payload, size_t payload_len, EaglePacket &out);
