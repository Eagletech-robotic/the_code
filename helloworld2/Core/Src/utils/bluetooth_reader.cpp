#include <array>
#include <cstdint>
#include <cstring>

constexpr int PACKET_SIZE = 20;

constexpr int PACKET_STARTER_BYTE = 255;

uint8_t packet0[PACKET_SIZE];
uint8_t packet1[PACKET_SIZE];

int ready_packet = -1;
int current_packet = 0;
int byte_index = 0;

uint8_t* get_current_packet() { return (current_packet == 0) ? (packet0) : (packet1); }

void byte_read(const uint8_t byte) {
    if (byte_index == 0 && byte != PACKET_STARTER_BYTE) return;

    uint8_t* current_packet_ptr = (current_packet == 0) ? (packet0) : (packet1);
    current_packet_ptr[byte_index] = byte;

    byte_index++;

    if (byte_index == PACKET_SIZE) {
        byte_index = 0;
        ready_packet = current_packet;
        current_packet = 1 - current_packet;
    }
}

bool is_packet_valid(const uint8_t* packet) {
    const uint8_t expected_checksum = packet[PACKET_SIZE - 1];

    uint8_t checksum = 0;
    for (int i = 1; i < PACKET_SIZE - 1; i++) {
        checksum += packet[i];
    }

    return (checksum & 127) == expected_checksum;
};

bool get_packet(std::array<uint8_t, PACKET_SIZE>& dest_packet) {
    if (ready_packet == -1) return false;

    uint8_t* packet = (ready_packet == 0) ? (packet0) : (packet1);
    if (!is_packet_valid(packet)) return false;

    std::memcpy(dest_packet.data(), packet, PACKET_SIZE);

    ready_packet = -1;

    return true;
}
