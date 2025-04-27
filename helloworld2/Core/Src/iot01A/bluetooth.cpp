#include <array>
#include <cstdint>
#include <cstring>

constexpr int STARTER_BYTE = 255;

constexpr int PACKET_SIZE = 20; // Number of bytes in a packet
constexpr int NB_PACKETS = 10;  // Number of packets to store

uint8_t packets[NB_PACKETS][PACKET_SIZE];

enum { STATE_WAITING_STARTER_BYTE, STATE_READING_PACKET };

static int bluetooth_state;
static int decoding_packet = 0;   // The packet currently being received.
static int byte_index = 0;        // The currently written byte in the decoding packet.
static int last_packet_read = -1; // The latest packet read, or -1 if no packet has been read yet.

bool is_packet_valid(const uint8_t *packet) {
    const uint8_t expected_checksum = packet[PACKET_SIZE - 1];

    uint8_t checksum = 0;
    for (int i = 1; i < PACKET_SIZE - 1; i++) {
        checksum += packet[i];
    }

    return (checksum & 127) == expected_checksum;
};

void bluetooth_decode(const uint8_t byte) {
    switch (bluetooth_state) {
    case STATE_WAITING_STARTER_BYTE: {
        if (byte == STARTER_BYTE) {
            bluetooth_state = STATE_READING_PACKET;
        }
        return;
    }
    case STATE_READING_PACKET: {
        uint8_t *packet_ptr = packets[decoding_packet];
        packet_ptr[byte_index] = byte;
        byte_index++;

        if (byte_index == PACKET_SIZE) {
            bluetooth_state = STATE_WAITING_STARTER_BYTE;
            byte_index = 0;
            if (is_packet_valid(packet_ptr)) {
                decoding_packet = (decoding_packet + 1) % NB_PACKETS;
            }
        }
    }
    }
}

bool read_packet(uint8_t (&out_packet)[PACKET_SIZE]) {
    int oldest_unread_packet = (last_packet_read + 1) % NB_PACKETS; // NB: will be 0 if last_packet_read was -1

    if (oldest_unread_packet == decoding_packet) {
        // No new packet available
        return false;
    } else {
        // Read the packet
        std::memcpy(out_packet, packets[oldest_unread_packet], PACKET_SIZE);
        last_packet_read = oldest_unread_packet;
        return true;
    }
}
