#include "robotic/bluetooth.hpp"
#include "utils/myprintf.hpp"

#include <cstring>

// Constructor
BluetoothDecoder::BluetoothDecoder() = default;

// Check the packet's checksum
bool BluetoothDecoder::is_packet_valid(const uint8_t *packet_data, const uint8_t checksum_byte) {
    uint8_t checksum = 0;
    for (int i = 0; i < PACKET_SIZE; ++i) {
        checksum += packet_data[i];
    }
    return (checksum & 0xFF) == checksum_byte;
}

// Process a single incoming byte
void BluetoothDecoder::byte_received(uint8_t byte) {
    uint8_t *packet_ptr = packets_buffer[current_packet_idx];

    switch (state) {
    case STATE_WAITING_STARTER_BYTE: {
        // Discard bytes until starter is found
        if (byte == STARTER_BYTE) {
            state = STATE_READING_PACKET;
            current_byte_idx = 0; // Reset byte index for the new packet
        }
        break;
    }
    case STATE_READING_PACKET: {
        packet_ptr[current_byte_idx++] = byte;

        // Check if the expected number of data bytes has been received
        if (current_byte_idx == PACKET_SIZE) {
            state = STATE_WAITING_CHECKSUM;
        }
        break;
    }
    case STATE_WAITING_CHECKSUM: {
        // The 'byte' received now is the checksum
        if (BluetoothDecoder::is_packet_valid(packet_ptr, byte)) {
            // Packet is valid, mark it ready by advancing the write pointer (current_packet_idx)
            current_packet_idx = (current_packet_idx + 1) % NB_PACKETS;

            // Check if the next write position will overwrite the oldest readable packet.
            // Advance the read pointer if we are about to overwrite.
            int oldest_unread_packet_idx = (last_read_packet_idx + 1) % NB_PACKETS;
            if (current_packet_idx == oldest_unread_packet_idx) {
                last_read_packet_idx = oldest_unread_packet_idx;
            }
        }

        // Reset for the next packet, regardless of checksum validity
        state = STATE_WAITING_STARTER_BYTE;
        break;
    }
    }
}

// Read the oldest available packet. Return nullptr if no packet is available.
// IMPORTANT: The packet will eventually be overwritten by the next incoming packet, handle it quickly.
uint8_t *BluetoothDecoder::read_packet() {
    int const oldest_unread_packet_idx = (last_read_packet_idx + 1) % NB_PACKETS;

    if (oldest_unread_packet_idx == current_packet_idx) {
        // No new packet available
        return nullptr;
    } else {
        last_read_packet_idx = oldest_unread_packet_idx;
        return packets_buffer[oldest_unread_packet_idx];
    }
}

// --- Single instance ---
BluetoothDecoder g_bluetooth_decoder;
