#include "robotic/bluetooth.hpp"
#include <gtest/gtest.h>
#include <numeric> // For std::iota
#include <vector>

// Test fixture for BluetoothDecoder tests
// We don't strictly *need* a fixture now since the state is encapsulated,
// but it can be useful for helper methods or common setup later.
class BluetoothDecoderTest : public ::testing::Test {
  protected:
    BluetoothDecoder decoder; // Each test gets a fresh decoder instance

    // Helper function to calculate checksum for a given data vector
    static uint8_t calculate_checksum(const std::vector<uint8_t> &data) {
        uint8_t checksum = 0;
        for (uint8_t byte : data) {
            checksum += byte;
        }
        return checksum & 0xFF;
    }

    // Helper function to feed a sequence of bytes to the decoder
    void feedBytes(const std::vector<uint8_t> &bytes) {
        for (uint8_t b : bytes) {
            decoder.byte_received(b);
        }
    }
};

// Test case: Initial state - should not have any packets ready
TEST_F(BluetoothDecoderTest, InitialStateNoPacket) {
    uint8_t packet_buffer[PACKET_SIZE];
    EXPECT_FALSE(decoder.read_packet(packet_buffer));
}

// Test case: Feed a single valid packet
TEST_F(BluetoothDecoderTest, ReceiveSingleValidPacket) {
    uint8_t packet_buffer[PACKET_SIZE];
    std::vector<uint8_t> test_data(PACKET_SIZE);
    // Fill with some simple data, e.g., 0, 1, 2, ...
    std::iota(test_data.begin(), test_data.end(), 0);
    uint8_t checksum = calculate_checksum(test_data);

    // Construct the full byte stream: Starter, Data, Checksum
    std::vector<uint8_t> byte_stream;
    byte_stream.push_back(BluetoothDecoder::STARTER_BYTE);
    byte_stream.insert(byte_stream.end(), test_data.begin(), test_data.end());
    byte_stream.push_back(checksum);

    // Feed the bytes
    feedBytes(byte_stream);

    // Check if a packet is ready and if it matches the sent data
    ASSERT_TRUE(decoder.read_packet(packet_buffer)) << "Decoder should have a packet ready";

    // Compare buffer content with original data
    for (int i = 0; i < PACKET_SIZE; ++i) {
        EXPECT_EQ(packet_buffer[i], test_data[i]) << "Packet data mismatch at index " << i;
    }

    // Check that no more packets are available
    EXPECT_FALSE(decoder.read_packet(packet_buffer)) << "Should be no more packets after reading one";
}

// Test case: Feed valid starter but incomplete packet
TEST_F(BluetoothDecoderTest, IncompletePacket) {
    uint8_t packet_buffer[PACKET_SIZE];
    std::vector<uint8_t> byte_stream;
    byte_stream.push_back(BluetoothDecoder::STARTER_BYTE);
    // Send only half the data bytes
    for (int i = 0; i < PACKET_SIZE / 2; ++i) {
        byte_stream.push_back(i);
    }

    feedBytes(byte_stream);
    EXPECT_FALSE(decoder.read_packet(packet_buffer)) << "Packet should not be ready yet";
}

// Test case: Feed packet with incorrect checksum
TEST_F(BluetoothDecoderTest, InvalidChecksum) {
    uint8_t packet_buffer[PACKET_SIZE];
    std::vector<uint8_t> test_data(PACKET_SIZE);
    std::iota(test_data.begin(), test_data.end(), 5); // Data 5, 6, ...
    uint8_t correct_checksum = calculate_checksum(test_data);
    uint8_t wrong_checksum = correct_checksum + 1;

    std::vector<uint8_t> byte_stream;
    byte_stream.push_back(BluetoothDecoder::STARTER_BYTE);
    byte_stream.insert(byte_stream.end(), test_data.begin(), test_data.end());
    byte_stream.push_back(wrong_checksum); // Send incorrect checksum

    feedBytes(byte_stream);

    // Packet should be rejected due to checksum, buffer should remain empty
    EXPECT_FALSE(decoder.read_packet(packet_buffer)) << "Packet with invalid checksum should be discarded";
}

// Test case: Feed multiple valid packets and read them
TEST_F(BluetoothDecoderTest, ReceiveMultipleValidPackets) {
    uint8_t packet_buffer[PACKET_SIZE];

    // Packet 1
    std::vector<uint8_t> data1(PACKET_SIZE);
    std::iota(data1.begin(), data1.end(), 10); // 10, 11, ...
    uint8_t cs1 = calculate_checksum(data1);

    // Packet 2
    std::vector<uint8_t> data2(PACKET_SIZE);
    std::iota(data2.begin(), data2.end(), 20); // 20, 21, ...
    uint8_t cs2 = calculate_checksum(data2);

    // Stream: P1, P2
    std::vector<uint8_t> stream;
    stream.push_back(BluetoothDecoder::STARTER_BYTE);
    stream.insert(stream.end(), data1.begin(), data1.end());
    stream.push_back(cs1);
    stream.push_back(BluetoothDecoder::STARTER_BYTE);
    stream.insert(stream.end(), data2.begin(), data2.end());
    stream.push_back(cs2);

    feedBytes(stream);

    // Read Packet 1
    ASSERT_TRUE(decoder.read_packet(packet_buffer));
    for (int i = 0; i < PACKET_SIZE; ++i)
        EXPECT_EQ(packet_buffer[i], data1[i]);

    // Read Packet 2
    ASSERT_TRUE(decoder.read_packet(packet_buffer));
    for (int i = 0; i < PACKET_SIZE; ++i)
        EXPECT_EQ(packet_buffer[i], data2[i]);

    // No more packets
    EXPECT_FALSE(decoder.read_packet(packet_buffer));
}

// Test case: Interspersed noise between valid packets
TEST_F(BluetoothDecoderTest, NoiseBetweenPackets) {
    uint8_t packet_buffer[PACKET_SIZE];

    // Packet 1
    std::vector<uint8_t> data1(PACKET_SIZE);
    std::iota(data1.begin(), data1.end(), 30);
    uint8_t cs1 = calculate_checksum(data1);

    // Stream: Noise, P1, Noise
    std::vector<uint8_t> stream = {0x11, 0x22, 0x33}; // Noise
    stream.push_back(BluetoothDecoder::STARTER_BYTE);
    stream.insert(stream.end(), data1.begin(), data1.end());
    stream.push_back(cs1);
    stream.push_back(0xAA); // More noise
    stream.push_back(0xBB);

    feedBytes(stream);

    // Read Packet 1
    ASSERT_TRUE(decoder.read_packet(packet_buffer));
    for (int i = 0; i < PACKET_SIZE; ++i)
        EXPECT_EQ(packet_buffer[i], data1[i]);

    // No more packets expected
    EXPECT_FALSE(decoder.read_packet(packet_buffer));
}

// Test case: Buffer wrapping (requires NB_PACKETS + 1 packets)
TEST_F(BluetoothDecoderTest, BufferWrapping) {
    uint8_t packet_buffer[PACKET_SIZE];
    const int num_packets_to_send = BluetoothDecoder::NB_PACKETS + 1;

    // Send NB_PACKETS + 1 packets
    for (int p = 0; p < num_packets_to_send; ++p) {
        std::vector<uint8_t> data(PACKET_SIZE);
        // Make data unique for each packet: p*100 + 0, p*100 + 1, ...
        std::iota(data.begin(), data.end(), static_cast<uint8_t>(p * 100));
        uint8_t cs = calculate_checksum(data);

        feedBytes({BluetoothDecoder::STARTER_BYTE});
        feedBytes(data);
        feedBytes({cs});
    }

    // Now, try to read. We should only be able to read the *last* NB_PACKETS-1 packets.
    // The first packet sent should have been overwritten.
    for (int p = 2; p < num_packets_to_send; ++p) { // Start from p=2 (the third packet sent)
        std::vector<uint8_t> expected_data(PACKET_SIZE);
        std::iota(expected_data.begin(), expected_data.end(), static_cast<uint8_t>(p * 100));

        ASSERT_TRUE(decoder.read_packet(packet_buffer))
            << "Failed reading packet corresponding to original index " << p;
        for (int i = 0; i < PACKET_SIZE; ++i) {
            EXPECT_EQ(packet_buffer[i], expected_data[i])
                << "Data mismatch in packet for original index " << p << " at byte index " << i;
        }
    }

    // After reading NB_PACKETS - 1, the buffer should be empty
    EXPECT_FALSE(decoder.read_packet(packet_buffer));
}
