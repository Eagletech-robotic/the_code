/*
 * um7.c
 *
 *  Created on: Feb 11, 2025
 *      Author: nboulay
 */

// Drivers UM7
// https://www.pololu.com/file/0J1556/UM7%20Datasheet_v1-8_30.07.2018.pdf
// code issue : https://github.com/Nagillimi/MYUM7/blob/master/MYUM7.cpp

#include<stdint.h>
#include "robotic/um7.h"
typedef int byte;

// Placeholder for parsing binary packets
int state;
	// Unscoped enumeration
enum {STATE_ZERO,STATE_S,STATE_SN,STATE_SNP,STATE_PT,STATE_DATA,STATE_CHK1,STATE_CHK0};
uint32_t error;
// EULER Variables
	int16_t roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
	float euler_time;

	// QUATERNION Variables
	int16_t quat_a, quat_b, quat_c, quat_d, quat_time;

	// RAW Variables
	int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;
	int16_t accel_raw_x, accel_raw_y, accel_raw_z;
	int16_t mag_raw_x, mag_raw_y, mag_raw_z;
	float temp, temp_time;

	float gyro_raw_time, accel_raw_time, mag_raw_time;

	// PROCESSED Variables
	float gyro_x, gyro_y, gyro_z, gyro_time;
	float accel_x, accel_y, accel_z, accel_time;
	float mag_x, mag_y, mag_z, mag_time;

	// POSITION and VELOCITY Variables
	float north_pos, east_pos, up_pos, pos_time;
	float north_vel, east_vel, up_vel, vel_time;

	// GPS Variables
	// Only available if GPS is installed with coms set on TX2/RX2
	float lattitude, longitude, altitude, course, speed, gps_time;

	// SAT Variables
	// Only available if GPS is installed with coms set on TX2/RX2
	// SNR = Signal-to-Noise Ratio
	// (Note index is 1 lower than actual satellite ID)
	float satellite_id[12], satellite_SNR[12];

	// GYRO BIAS Variables.
	// Not necessary to read in for ZERO_GYROS, that function already measures these
	float gyro_bias_x, gyro_bias_y, gyro_bias_z;

// Functional Variables
byte packet_type;
byte address;
bool packet_is_batch;
byte batch_length;
bool packet_has_data;
byte data[52];
byte data_length;
byte data_index;
byte cmd_buffer[7];
byte config_buffer[11];
char firmware[4];
byte checksum1;		                            // First byte of checksum
byte checksum0;		                            // Second byte of checksum
uint16_t checksummer  = (checksum1<<8) | checksum0; // Combine the checksums
unsigned short checksum10;			    // Checksum received from packet
unsigned short computed_checksum;	            // Checksum computed from bytes received

typedef union {
	float val;
	uint8_t bytes[4];
} floatval;

/*

*/
union combine {
	float f;
	uint8_t b[4];
};

float read_register_as_float(int firstByte) { // For one register as an IEEE floatpoint
	floatval temp;
	temp.bytes[3] = data[(firstByte)];
	temp.bytes[2] = data[(firstByte + 1)];
	temp.bytes[1] = data[(firstByte + 2)];
	temp.bytes[0] = data[(firstByte + 3)];
	return temp.val;
}

void um7_save() {
	switch (address) {

	case DREG_HEALTH:
		error = read_register_as_float(0);
	break;


	case DREG_GYRO_RAW_XY : // In 2's complement!
		if (packet_is_batch) {
			gyro_raw_x = (uint16_t)((data[0] << 8) | data[1]);
			gyro_raw_y = (uint16_t)((data[2] << 8) | data[3]);
			gyro_raw_z = (uint16_t)((data[4] << 8) | data[5]);
			gyro_raw_time = read_register_as_float(6);

			accel_raw_x = (uint16_t)((data[10] << 8) | data[11]);
			accel_raw_y = (uint16_t)((data[12] << 8) | data[13]);
			accel_raw_z = (uint16_t)((data[14] << 8) | data[15]);
			accel_raw_time = read_register_as_float(16);

			mag_raw_x = (uint16_t)((data[20] << 8) | data[21]);
			mag_raw_y = (uint16_t)((data[22] << 8) | data[23]);
			mag_raw_z = (uint16_t)((data[24] << 8) | data[25]);
			mag_raw_time = read_register_as_float(26);

			temp = read_register_as_float(30);
			temp_time = read_register_as_float(34);
		} else {
			gyro_x = (int16_t)((data[0] << 8) | data[1]);
		}
    break;


	case DREG_GYRO_PROC_X : // CALIBRATION has to be applied first!! Typicaly done in-house @CHR
		if (packet_is_batch) {
			gyro_x = read_register_as_float(0);
			gyro_y = read_register_as_float(4);
			gyro_z = read_register_as_float(8);
			gyro_time = read_register_as_float(12);

			accel_x = read_register_as_float(16);
			accel_y = read_register_as_float(20);
			accel_z = read_register_as_float(24);
			accel_time = read_register_as_float(28);

			mag_x = read_register_as_float(32);
			mag_y = read_register_as_float(36);
			mag_z = read_register_as_float(40);
			mag_time = read_register_as_float(44);
		} else {
			gyro_x = read_register_as_float(0);
		}
    break;


	case DREG_QUAT_AB : // NOT a priority!!
		if (packet_is_batch) {
		  quat_a = (int16_t)((data[0]<<8) | data[1]) / 29789.09091;
		  quat_b = (int16_t)((data[2]<<8) | data[3]) / 29789.09091;
		  quat_c = (int16_t)((data[4]<<8) | data[5]) / 29789.09091;
		  quat_d = (int16_t)((data[6]<<8) | data[7]) / 29789.09091;
		  quat_time = read_register_as_float(8);
		} else {
		  quat_a = (int16_t)((data[0]<<8) | data[1]) / 29789.09091;
		  quat_b = (int16_t)((data[2]<<8) | data[3]) / 29789.09091;
		}
    break;


	case DREG_EULER_PHI_THETA : // data[6] and data[7] are unused. WORKS!!
  		if (packet_is_batch) {
			roll  = (int16_t)((data[0]<<8) | data[1]) / 91.02222;
    		pitch = (int16_t)((data[2]<<8) | data[3]) / 91.02222;
  			yaw   = (int16_t)((data[4]<<8) | data[5]) / 91.02222;
			roll_rate  = (int16_t)((data[8] << 8) | data[9]) / 16.0;
			pitch_rate = (int16_t)((data[10] << 8) | data[11]) / 16.0;
			yaw_rate   = (int16_t)((data[12] << 8) | data[13]) / 16.0;
			euler_time = read_register_as_float(14);
  		} else {
			roll  = (int16_t)((data[0]<<8) | data[1]) / 91.02222;
			pitch = (int16_t)((data[2]<<8) | data[3]) / 91.02222;
  		}
  	break;


	case DREG_POSITION_N :
		if (packet_is_batch) {
			north_pos = read_register_as_float(0);
			east_pos = read_register_as_float(4);
			up_pos = read_register_as_float(8);
			pos_time = read_register_as_float(12);
		}
		else {
			north_pos = read_register_as_float(0);
		}
	break;


	case DREG_VELOCITY_N :
		if (packet_is_batch) {
			north_vel = read_register_as_float(0);
			east_vel = read_register_as_float(4);
			up_vel = read_register_as_float(8);
			vel_time = read_register_as_float(12);
		}
		else {
			north_vel = read_register_as_float(0);
		}
	break;


	case DREG_GPS_LATITUDE : // Only available if GPS is installed with coms set on TX2/RX2
		if (packet_is_batch) {
			lattitude = read_register_as_float(0);
			longitude = read_register_as_float(4);
			altitude = read_register_as_float(8);
			course = read_register_as_float(12);
			speed = read_register_as_float(16);
			gps_time = read_register_as_float(20);
		}
		else {
			lattitude = read_register_as_float(0);
		}
	break;


	case DREG_GPS_SAT_1_2 : // Only available if GPS is installed with coms set on TX2/RX2
		// Satellite 1 is satellite_id[0]
		if (packet_is_batch) {
			satellite_id[0] = (uint8_t)(data[0]);
			satellite_SNR[0] = (uint8_t)(data[1]);
			satellite_id[1] = (uint8_t)(data[2]);
			satellite_SNR[1] = (uint8_t)(data[3]);
			satellite_id[2] = (uint8_t)(data[4]);
			satellite_SNR[2] = (uint8_t)(data[5]);
			satellite_id[3] = (uint8_t)(data[6]);
			satellite_SNR[3] = (uint8_t)(data[7]);
			satellite_id[4] = (uint8_t)(data[8]);
			satellite_SNR[4] = (uint8_t)(data[9]);
			satellite_id[5] = (uint8_t)(data[10]);
			satellite_SNR[5] = (uint8_t)(data[11]);
			satellite_id[6] = (uint8_t)(data[12]);
			satellite_SNR[6] = (uint8_t)(data[13]);
			satellite_id[7] = (uint8_t)(data[14]);
			satellite_SNR[7] = (uint8_t)(data[15]);
			satellite_id[8] = (uint8_t)(data[16]);
			satellite_SNR[8] = (uint8_t)(data[17]);
			satellite_id[9] = (uint8_t)(data[18]);
			satellite_SNR[9] = (uint8_t)(data[19]);
			satellite_id[10] = (uint8_t)(data[20]);
			satellite_SNR[10] = (uint8_t)(data[21]);
			satellite_id[11] = (uint8_t)(data[22]);
			satellite_SNR[11] = (uint8_t)(data[23]);
		}
		else {
			satellite_id[0] = (uint8_t)(data[0]);
			satellite_SNR[0] = (uint8_t)(data[1]);
			satellite_id[1] = (uint8_t)(data[2]);
			satellite_SNR[1] = (uint8_t)(data[3]);
		}
	break;


	case DREG_GYRO_BIAS_X :
		if (packet_is_batch) {
			gyro_bias_x = read_register_as_float(0);
			gyro_bias_y = read_register_as_float(4);
			gyro_bias_z = read_register_as_float(8);
		}
		else {
			gyro_bias_x = read_register_as_float(0);
		}
	break;
	}
}


bool um7_checksum() {
	checksum10 = ((checksum1 << 8) | checksum0);	// Combine checksum1 and checksum0
	computed_checksum = 's' + 'n' + 'p' + packet_type + address;
	for (int i = 0; i < data_length; i++) { // computed_checksum can only be 16bits long (2B)
		computed_checksum += data[i];
	}
	if (checksum10 == computed_checksum) {
		um7_save();
		return true;
	} else {
		return false;
	}
}


int um7_decode(int current_byte) {

	switch(state) {
	case STATE_ZERO:
		if (current_byte == 's') {
			state = STATE_S;		// Entering state S from state Zero
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_S:
		if (current_byte == 'n') {
			state = STATE_SN;		// Entering state SN from state S
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_SN:
		if (current_byte == 'p') {
			state = STATE_SNP;		// Entering state SNP from state SN.  Packet header detected.
		} else {
			state = STATE_ZERO;
		}
		return false;
	case STATE_SNP:
		state = STATE_PT;			// Entering state PT from state SNP.  Decode packet type.
		packet_type = current_byte;
		packet_has_data = (packet_type >> 7) & 0x01;
		packet_is_batch = (packet_type >> 6) & 0x01;
		batch_length    = (packet_type >> 2) & 0x0F;
		if (packet_has_data) {
			if (packet_is_batch) {
				data_length = 4 * batch_length;	// Each data packet is 4 bytes long
			} else {
				data_length = 4;
			}
		} else {
			data_length = 0;
		}
		return false;
	case STATE_PT:
		state = STATE_DATA;		// Next state will be READ_DATA.  Save address to memory. (eg 0x70 for a DREG_EULER_PHI_THETA packet)
		address = current_byte;
		data_index = 0;
		return false;
	case STATE_DATA:			//  Entering state READ_DATA.  Stay in state until all data is read.
		data[data_index] = current_byte;
		data_index++;
		if (data_index >= data_length){
			state = STATE_CHK1;	//  Data read completed.  Next state will be CHK1
		}
		return false;
	case STATE_CHK1:			// Entering state CHK1, save the byte as checksum1.  Next state will be CHK0
		state = STATE_CHK0;
		checksum1 = current_byte;
		return false;
	case STATE_CHK0:
		state = STATE_ZERO;		// Entering state CHK0, save the byte as checksum0.  Next state will be state Zero.
		checksum0 = current_byte;
		return um7_checksum();
	}
	return false;
}


//
//
//
// Orientation yaw pitch roll
// pos : north_pos, east_pos, up_pos,

void um7_get_pos(um7_t *um7) {
	um7->east_pos = east_pos;
	um7->north_pos = north_pos;
	um7->up_pos = up_pos;
	um7->yaw = yaw;
	um7->roll = roll;
	um7->pitch = pitch;
}
