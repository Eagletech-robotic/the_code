#pragma once

#include <cstdint>

constexpr int PACKET_SIZE = 20;

void bluetooth_decode(uint8_t byte);
bool read_packet(uint8_t out_packet[]);
