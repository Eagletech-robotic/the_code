#pragma once
#include "main.h"
#include <stdint.h>
//////////////////////////////////////
//      CONFIGURATION REGISTERS	    //
//////////////////////////////////////

#define CREG_COM_SETTINGS 0x00  // Baud rates for reading over the UM7, default 115200 baud
#define CREG_COM_RATES1 0x01    // Individual raw data rate
#define CREG_COM_RATES2 0x02    // ALL raw data rate
#define CREG_COM_RATES3 0x03    // Individual processed data rate
#define CREG_COM_RATES4 0x04    // ALL processed data rate
#define CREG_COM_RATES5 0x05    // Quat, Euler, position, and velocity data rate
#define CREG_COM_RATES6 0x06    // Pose (euler & position), health, and gyro bias estimate rates
#define CREG_COM_RATES7 0x07    // Sets data rate for CHR NMEA-style packets
#define CREG_MISC_SETTINGS 0x08 // Contains filter and sensor control options

#define CREG_HOME_NORTH 0x09 // Sets north from current position
#define CREG_HOME_EAST 0x0A  // Sets east from current position
#define CREG_HOME_UP 0x0B    // Sets home altitude in meters

#define CREG_GYRO_TRIM_X 0x0C
#define CREG_GYRO_TRIM_Y 0x0D
#define CREG_GYRO_TRIM_Z 0x0E

#define CREG_MAG_CAL1_1 0x0F
#define CREG_MAG_CAL1_2 0x10
#define CREG_MAG_CAL1_3 0x11
#define CREG_MAG_CAL2_1 0x12
#define CREG_MAG_CAL2_2 0x13
#define CREG_MAG_CAL2_3 0x14
#define CREG_MAG_CAL3_1 0x15
#define CREG_MAG_CAL3_2 0x16
#define CREG_MAG_CAL3_3 0x17

#define CREG_MAG_BIAS_X 0x18
#define CREG_MAG_BIAS_Y 0x19
#define CREG_MAG_BIAS_Z 0x1A

#define CREG_ACCEL_CAL1_1 0x1B
#define CREG_ACCEL_CAL1_2 0x1C
#define CREG_ACCEL_CAL1_3 0x1D
#define CREG_ACCEL_CAL2_1 0x1E
#define CREG_ACCEL_CAL2_2 0x1F
#define CREG_ACCEL_CAL2_3 0x20
#define CREG_ACCEL_CAL3_1 0x21
#define CREG_ACCEL_CAL3_2 0x22
#define CREG_ACCEL_CAL3_3 0x23

#define CREG_ACCEL_BIAS_X 0x24
#define CREG_ACCEL_BIAS_Y 0x25
#define CREG_ACCEL_BIAS_Z 0x26

//////////////////////////////
//	DATA REGISTERS	    //
//////////////////////////////

#define DREG_HEALTH 0x55
#define DREG_GYRO_RAW_XY 0x56
#define DREG_GYRO_RAW_Z 0x57
#define DREG_GYRO_RAW_TIME 0x58
#define DREG_ACCEL_RAW_XY 0x59
#define DREG_ACCEL_RAW_Z 0x5A
#define DREG_ACCEL_RAW_TIME 0x5B
#define DREG_MAG_RAW_XY 0x5C
#define DREG_MAG_RAW_Z 0x5D
#define DREG_MAG_RAW_TIME 0x5E
#define DREG_TEMPERATURE 0x5F
#define DREG_TEMPERATURE_TIME 0x60

#define DREG_GYRO_PROC_X 0x61     // deg/s
#define DREG_GYRO_PROC_Y 0x62     // deg/s
#define DREG_GYRO_PROC_Z 0x63     // deg/s
#define DREG_GYRO_PROC_TIME 0x64  // time
#define DREG_ACCEL_PROC_X 0x65    // m/s/s
#define DREG_ACCEL_PROC_Y 0x66    // m/s/s
#define DREG_ACCEL_PROC_Z 0x67    // m/s/s
#define DREG_ACCEL_PROC_TIME 0x68 // time
#define DREG_MAG_PROC_X 0x69      // T
#define DREG_MAG_PROC_Y 0x6A      // T
#define DREG_MAG_PROC_Z 0x6B      // T
#define DREG_MAG_PROC_TIME 0x6C   // time

#define DREG_QUAT_AB 0x6D             // both in attitude
#define DREG_QUAT_CD 0x6E             // both in attitude
#define DREG_QUAT_TIME 0x6F           // time
#define DREG_EULER_PHI_THETA 0x70     // both in deg
#define DREG_EULER_PSI 0x71           // deg
#define DREG_EULER_PHI_THETA_DOT 0x72 // both in deg/s
#define DREG_EULER_PSI_DOT 0x73       // deg/s
#define DREG_EULER_TIME 0x74          // time
#define DREG_POSITION_N 0x75          //
#define DREG_POSITION_E 0x76
#define DREG_POSITION_UP 0x77
#define DREG_POSITION_TIME 0x78
#define DREG_VELOCITY_N 0x79
#define DREG_VELOCITY_E 0x7A
#define DREG_VELOCITY_UP 0x7B
#define DREG_VELOCITY_TIME 0x7C

#define DREG_GPS_LATITUDE 0x7D
#define DREG_GPS_LONGITUDE 0x7E
#define DREG_GPS_ALTITUDE 0x7F
#define DREG_GPS_COURSE 0x80
#define DREG_GPS_SPEED 0x81
#define DREG_GPS_TIME 0x82
#define DREG_GPS_SAT_1_2 0x83
#define DREG_GPS_SAT_3_4 0x84
#define DREG_GPS_SAT_5_6 0x85
#define DREG_GPS_SAT_7_8 0x86
#define DREG_GPS_SAT_9_10 0x87
#define DREG_GPS_SAT_11_12 0x88

#define DREG_GYRO_BIAS_X 0x89
#define DREG_GYRO_BIAS_Y 0x8A
#define DREG_GYRO_BIAS_Z 0x8B

//////////////////////////////////
//	COMMAND REGISTERS	//
//////////////////////////////////

#define GET_FW_REVISION 0xAA
#define FLASH_COMMIT                                                                                                   \
    0xAB // Causes the UM7 to write all configuration settings to FLASH so that they will remain when the power is
         // cycled.
#define RESET_TO_FACTORY 0xAC
#define ZERO_GYROS                                                                                                     \
    0xAD // Measures the gyro outputs and sets the output trim registers to compensate for any non-zero bias. Keep flat.
#define SET_HOME_POSITION 0xAE
#define SET_MAG_REFERENCE 0xB0
#define CALIBRATE_ACCELEROMETERS                                                                                       \
    0xB1 // Reboots the UM7 and performs a crude calibration on the accelerometers. Keep flat.
#define RESET_EKF 0xB3

typedef struct um7_t {
    float yaw;
    float pitch;
    float roll;
    float north_pos;
    float east_pos;
    float up_pos;
    float north_v;
    float east_v;
    float up_v;
    float accel_x; // vers l'avant
    float accel_y; // vers la gauche
    float accel_z; // vers le haut
} um7_t;

int um7_decode(int current_byte);
void um7_get_pos(um7_t *um7);
void um7_print(um7_t *um7);
void um7_set_all_processed_rate(UART_HandleTypeDef *huart, uint8_t rate);
void um7_set_position_rate(UART_HandleTypeDef *huart, uint8_t rate);
void um7_set_pose_rate(UART_HandleTypeDef *huart, uint8_t rate);
void um7_set_home_position(UART_HandleTypeDef *huart);
void um7_zero_gyros(UART_HandleTypeDef *huart);
void um7_set_mag_reference(UART_HandleTypeDef *huart);
void um7_calibrate_accelerometers(UART_HandleTypeDef *huart);
void um7_factory_reset(UART_HandleTypeDef *huart);
void um7_reset_kalman_filter(UART_HandleTypeDef *huart);
void um7_set_misc_settings(UART_HandleTypeDef *huart, int pps, int zg, int q, int mag);
void um7_set_home_north(UART_HandleTypeDef *huart, float north);
void um7_set_home_north_current(UART_HandleTypeDef *huart);
