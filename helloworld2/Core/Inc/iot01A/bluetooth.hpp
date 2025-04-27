#pragma once

#include <array>
#include <cstdint>
#include <cstring>

constexpr int PACKET_SIZE = 20;
constexpr int STARTER_BYTE = 255;

extern uint8_t packet0[PACKET_SIZE];
extern uint8_t packet1[PACKET_SIZE];

enum { STATE_WAITING_STARTER_BYTE, STATE_READING_PACKET };

extern int bluetooth_state;
extern int current_packet; // The packet currently being received.
extern int byte_index;
extern int ready_packet_nb; // The latest packet received and not yet read. -1 if no packet is ready.

void bluetooth_decode(const uint8_t byte);
bool read_packet(uint8_t (&out_packet)[PACKET_SIZE]);
