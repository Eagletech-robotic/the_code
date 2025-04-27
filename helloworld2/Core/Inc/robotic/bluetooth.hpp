#pragma once

#include <stdint.h>

constexpr int PACKET_SIZE = 10; // Number of bytes in a packet

void bluetooth_decode(uint8_t byte);
bool read_packet(uint8_t out_packet[]);
