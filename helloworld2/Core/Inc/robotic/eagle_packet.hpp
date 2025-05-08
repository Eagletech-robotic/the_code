#pragma once
#include <cstddef>
#include <cstdint>

/* --------------------------------------------------------------------------
   Bluetooth packet format (little‑endian bit order).
   Packet length: 128 bytes.

   Byte   Bit‑index  Purpose                                Size (bits)
   ───────────────────────────────────────────────────────────────────────
   0-7   0          robot_colour (0=blue, 1=yellow)        1
         1          robot_detected (0=no, 1=yes)           1
         2‑10       robot_x (0–300)                        9
         11‑18      robot_y (0–200)                        8
         19‑27      robot_orientation_deg (-180...180)     9   (value+180)
         28         opponent_detected (0=no, 1=yes)        1
         29‑37      opponent_x (0–300)                     9
         38‑45      opponent_y (0–200)                     8
         46‑54      opponent_orientation_deg (‑180…180)    9   (value+180)
         55‑60      object_count (0–60)                    6
         61‑63      padding (0)                            3
   8+2i  repeating object (0 <= i < 60):                   16 bits each
         0-1        type (0=bleacher, 1=plank, 2=can)      2
         2-7        x (0–300)                              6
         8-12       y (0–200)                              5
         13-15      θ_deg (0,30,…,145)                     3
   ->127            padding (0)                            fill to 128 bytes
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
    int16_t robot_orientation_deg;

    bool opponent_detected;
    uint16_t opponent_x_cm;
    uint16_t opponent_y_cm;
    int16_t opponent_orientation_deg;

    uint8_t object_count;
    EagleObject objects[60];
};

// Decode the payload sitting between starter byte and checksum.
// Returns true on success; false if object_count > 60 or payload too short.
bool decode_eagle_packet(const uint8_t *payload, size_t payload_len, EaglePacket &out);
