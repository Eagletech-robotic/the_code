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

// Attempt to read the oldest available packet
bool BluetoothDecoder::read_packet(uint8_t out_packet[PACKET_SIZE]) {
    // Calculate the index of the oldest packet that hasn't been read yet
    int oldest_unread_packet_idx = (last_read_packet_idx + 1) % NB_PACKETS;

    // Check if the oldest unread packet index is the same as the current write index.
    // If they are the same, it means the buffer is empty (or all packets have been read).
    if (oldest_unread_packet_idx == current_packet_idx) {
        // No new packet available
        return false;
    } else {
        // Copy the oldest unread packet to the output buffer
        std::memcpy(out_packet, packets_buffer[oldest_unread_packet_idx], PACKET_SIZE); // Use member variable

        // Debug
        char output_str[PACKET_SIZE + 30]; // Extra space for the prefix message
        int str_pos = sprintf(output_str, "Packet %d read: ", oldest_unread_packet_idx);
        int i = 0;
        while (i < PACKET_SIZE)
            output_str[str_pos++] = out_packet[i++];
        output_str[str_pos] = '\0';
        myprintf("Read packet: %s", output_str);
        //  End of debug

        // Update the index of the last read packet
        last_read_packet_idx = oldest_unread_packet_idx;
        return true;
    }
}

// --- Single instance ---
BluetoothDecoder g_bluetooth_decoder;
