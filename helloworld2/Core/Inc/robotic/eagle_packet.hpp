#pragma once
#include <cstddef>
#include <cstdint>

/* --------------------------------------------------------------------------
   Bluetooth packet format (little‑endian bit order).
   Packet length: 128 bytes.

   Byte   Bit‑index  Purpose                                Size (bits)
   ───────────────────────────────────────────────────────────────────────
   0-7   0          robot_colour (0=blue, 1=yellow)        1
         1‑9        robot_x (0–300)                        9
         10‑17      robot_y (0–200)                        8
         18‑26      robot_orientation_deg (-180...180)     9   (value+180)
         27‑35      opponent_x (0–300)                     9
         36‑43      opponent_y (0–200)                     8
         44‑52      opponent_orientation_deg (‑180…180)    9   (value+180)
         53‑58      object_count (0–60)                    6
         59‑63      padding (0)                            5
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
    uint16_t robot_x_cm;
    uint16_t robot_y_cm;
    int16_t robot_orientation_deg;

    uint16_t opponent_x_cm;
    uint16_t opponent_y_cm;
    int16_t opponent_orientation_deg;

    uint8_t object_count;
    EagleObject objects[60];
};

// Decode the payload sitting between starter byte and checksum.
// Returns true on success; false if object_count > 60 or payload too short.
bool decode_eagle_packet(const uint8_t *payload, size_t payload_len, EaglePacket &out);
