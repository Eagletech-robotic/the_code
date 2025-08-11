#pragma once

#include <cstdint>

constexpr int PACKET_SIZE = 7;

class BluetoothDecoder {
  public:
    // Constants - needed by the test suite
    static constexpr int NB_PACKETS = 3;
    static constexpr int STARTER_BYTE = 0xFF;

    // Public Interface
    BluetoothDecoder();
    void byte_received(uint8_t byte);
    uint8_t *read_packet();

  private:
    // Internal State Machine
    enum State { STATE_WAITING_STARTER_BYTE, STATE_READING_PACKET, STATE_WAITING_CHECKSUM };
    State state = STATE_WAITING_STARTER_BYTE;

    uint8_t packets_buffer[NB_PACKETS][PACKET_SIZE]{}; // Buffer to store packets
    int current_packet_idx = 0;                        // Index of the packet buffer being written to
    int current_byte_idx = 0;                          // Index within the current packet being written
    int last_read_packet_idx = -1;                     // Index of the last packet read by the consumer

    // Private Helper Method
    static bool is_packet_valid(const uint8_t *packet_data, uint8_t checksum_byte);
};

extern BluetoothDecoder g_bluetooth_decoder;
