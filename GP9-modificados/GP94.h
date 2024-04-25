#ifndef GP94_H
#define GP94_H
#include "arduino.h"

  //////////////////////////////////////
 //      CONFIGURATION REGISTERS     //
//////////////////////////////////////

#define CREG_COM_SETTINGS 0x00 // Baud rates for reading over the GP9, default 115200 baud
#define CREG_COM_RATES1 0x01 // Individual raw data rate
#define CREG_COM_RATES2 0x02 // ALL raw data rate
#define CREG_COM_RATES3 0x03 // Individual processed data rate
#define CREG_COM_RATES4 0x04 // ALL processed data rate
#define CREG_COM_RATES5 0x05 // Quat, Euler, position, and velocity data rate
#define CREG_COM_RATES6 0x06 // Pose (euler & position), health, and gyro bias estimate rates
#define CREG_COM_RATES7 0x07 // Sets data rate for CHR NMEA-style packets
#define CREG_FILTER_SETTINGS 0x08 // Contains filter and sensor control options
#define CREG_HOME_NORTH 0x09 // Sets north from current position
#define CREG_HOME_EAST 0x0A // Sets east from current position
#define CREG_HOME_UP 0x0B // Sets home altitude in meters
#define CREG_ZERO_PRESSURE 0x0C // Sets barometric pressure at zero altitude
#define CREG_GYRO_TRIM_X 0x0E // Bias trim for x-axis rate gyro
#define CREG_GYRO_TRIM_Y 0x0F // Bias trim for y-axis rate gyro
#define CREG_GYRO_TRIM_Z 0x10 // Bias trim for z-axis rate gyro
#define CREG_MAG_CAL1_1 0x42 // Row 1, Column 1 of magnetometer calibration matrix
#define CREG_MAG_CAL1_2 0x43 // Row 1, Column 2 of magnetometer calibration matrix
#define CREG_MAG_CAL1_3 0x44 // Row 1, Column 3 of magnetometer calibration matrix
#define CREG_MAG_CAL2_1 0x45 // Row 2, Column 1 of magnetometer calibration matrix
#define CREG_MAG_CAL2_2 0x46 // Row 2, Column 2 of magnetometer calibration matrix
#define CREG_MAG_CAL2_3 0x47 // Row 2, Column 3 of magnetometer calibration matrix
#define CREG_MAG_CAL3_1 0x48 // Row 3, Column 1 of magnetometer calibration matrix
#define CREG_MAG_CAL3_2 0x49 // Row 3, Column 2 of magnetometer calibration matrix
#define CREG_MAG_CAL3_3 0x4A // Row 3, Column 3 of magnetometer calibration matrix
#define CREG_MAG_BIAS_X 0x4B // Magnetometer X-axis bias
#define CREG_MAG_BIAS_Y 0x4C // Magnetometer Y-axis bias
#define CREG_MAG_BIAS_Z 0x4D // Magnetometer Z-axis bias


  //////////////////////////////
 //     DATA REGISTERS       //
//////////////////////////////

#define DREG_HEALTH 0x55 // Contains information about the health and status of the GP9
#define DREG_GYRO_RAW_XY 0x56 // Raw X and Y rate gyro data
#define DREG_GYRO_RAW_Z 0x57 // Raw Z rate gyro data
#define DREG_GYRO_RAW_TIME 0x58 // Time at wich rate gyro data was acquired
#define DREG_ACCEL_RAW_XY 0x59 // Raw X and Y accelerometer data 
#define DREG_ACCEL_RAW_Z 0x5A // Raw Z accelerometer data 
#define DREG_ACCEL_RAW_TIME 0x5B // Time at wich accelerometer data was acquired
#define DREG_MAG_RAW_XY 0x5C // Raw X and Y magnetometer data 
#define DREG_MAG_RAW_Z 0x5D // Raw Z magnetometer data
#define DREG_MAG_RAW_TIME 0x5E // Time at wich magnetometer data was acquired
#define DREG_PRESSURE_RAW 0x5F // Raw absolute pressure data
#define DREG_PRESSURE_TIME 0x60 // Time at wich absolute pressure data was acquired
#define DREG_TEMPERATURE_RAW1 0X61 // Raw temperature data register
#define DREG_TEMPERATURE_RAW2 0X62 // Raw temperature data register
#define DREG_TEMPERATURE_TIME 0X63 // Time at wich temperature data was acquired
#define DREG_GYRO_PROC_X 0x64 // Processed x-axis rate gyro data (deg/s)
#define DREG_GYRO_PROC_Y 0x65 // Processed y-axis rate gyro data (deg/s)
#define DREG_GYRO_PROC_Z 0x66 // Processed z-axis rate gyro data (deg/s)
#define DREG_GYRO_PROC_TIME 0x67 // Time at wich rate gyro data was acquired
#define DREG_ACCEL_PROC_X 0x68 // Processed x-axis magnetometer data (m/s^2)
#define DREG_ACCEL_PROC_Y 0x69 // Processed y-axis magnetometer data (m/s^2)
#define DREG_ACCEL_PROC_Z 0x6A // Processed z-axis magnetometer data (m/s^2)
#define DREG_ACCEL_PROC_TIME 0x6B // Time at wich accelerometer data was acquired
#define DREG_MAG_PROC_X 0x6C // Processed x-axis magnetometer data
#define DREG_MAG_PROC_Y 0x6D // Processed y-axis magnetometer data
#define DREG_MAG_PROC_Z 0x6E // Processed z-axis magnetometer data
#define DREG_MAG_PROC_TIME 0x6F // Time at which the absolute pressure sensor was sampled.
#define DREG_PRESSURE_PROC 0X70 // Altitude in meters as measured by the absolute pressure sensor
#define DREG_PRESSURE_PROC_TIME 0X71 // Time at wich absolute pressure data was acquired
#define DREG_TEMPERATURE_PROC1 0X72 // Contains the actual temperature as reported by the gyro, accel, mag IC
#define DREG_TEMPERATURE_PROC2 0X73 // Contains the actual temperature from the pressure sensor.
#define DREG_TEMPERATURE_PROC_TIME 0X74 // Time at which temperature data was acquired
#define DREG_QUAT_AB 0x75 // Quaternion elements A and B
#define DREG_QUAT_CD 0x76 // Quaternion elements C and D
#define DREG_QUAT_TIME 0x77 // Time at which the sensor was at the specified quaternion rotation
#define DREG_EULER_PHI_THETA 0x78 // Roll and pitch angles (deg)
#define DREG_EULER_PSI 0x79 // Yaw angle (deg)
#define DREG_EULER_TIME 0x7A // Time of computed Euler attitude
#define DREG_POSITION_NORTH 0x7B // North position in meters
#define DREG_POSITION_EAST 0x7C // East position in meters
#define DREG_POSITION_UP 0x7D // Altitude in meters
#define DREG_POSITION_TIME 0x7E // Time of estimated position
#define DREG_VELOCITY_NORTH 0x7F // North velocity
#define DREG_VELOCITY_EAST 0x80 // East velocity
#define DREG_VELOCITY_UP 0x81 // Altitude velocity
#define DREG_VELOCITY_TIME 0x83 // Time of velocity estimate
#define DREG_GPS_LATITUDE 0x84 // GPS latitude
#define DREG_GPS_LONGITUDE 0x85 // GPS longitude
#define DREG_GPS_ALTITUDE 0x86 // GPS altitude
#define DREG_GPS_COURSE 0x87 // GPS course
#define DREG_GPS_SPEED 0x88 // GPS speed
#define DREG_GPS_TIME 0x89 // GPS time
#define DREG_GPS_DATE 0x8A // GPS date register
#define DREG_GPS_SAT_1_2 0x8B // GPS satellite information
#define DREG_GPS_SAT_3_4 0x8C // GPS satellite information
#define DREG_GPS_SAT_5_6 0x8D // GPS satellite information
#define DREG_GPS_SAT_7_8 0x8E // GPS satellite information
#define DREG_GPS_SAT_9_10 0x8F // GPS satellite information
#define DREG_GPS_SAT_11_12 0x90 // GPS satellite information
#define DREG_GYRO_BIAS_X 0x91 // X-axis gyro bias estimate
#define DREG_GYRO_BIAS_Y 0x92 // Y-axis gyro bias estimate
#define DREG_GYRO_BIAS_Z 0x93 // Z-axis gyro bias estimate
#define DREG_BIAS_X_VARIANCE 0x94 // Variance of gyro x-axis bias estimate
#define DREG_BIAS_Y_VARIANCE 0x95 // Variance of gyro y-axis bias estimate
#define DREG_BIAS_Z_VARIANCE 0x96 // Variance of gyro z-axis bias estimate
#define DREG_QUAT_A_VARIANCE 0x97 // Variance of quaternion element a
#define DREG_QUAT_B_VARIANCE 0x98 // Variance of quaternion element b
#define DREG_QUAT_C_VARIANCE 0x99 // Variance of quaternion element c
#define DREG_QUAT_D_VARIANCE 0x9A // Variance of quaternion element d

  ///////////////////////////////
 //     COMMAND REGISTERS     //
///////////////////////////////

#define GET_FW_REVISION 0xAA // Causes the GP9 to respond with a packet containing the current firmware revision.
#define FLASH_COMMIT 0xAB // Causes the GP9 to write all configuration settings to FLASH so that they will remain when the power is cycled.
#define RESET_TO_FACTORY 0xAC // Reset all settings to factory defaults
#define ZERO_GYROS 0xAD // Measures the gyro outputs and sets the output trim registers to compensate for any non-zero bias. Keep flat.
#define SET_HOME_POSITION 0xAE // Sets the current GPS location as position (0,0)
#define RESET_EKF 0xB3 // Resets the EKF

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdlib.h>

class GP9 {

public:
  GP9(HardwareSerial &serial); // Constructor
  HardwareSerial *serial_port;
  uint32_t error;

    //////////////////////////////////
   //     ACCESIBLE FUNCTIONS      //
  //////////////////////////////////

  bool decode(byte current_byte);
  void error_status(); // needs work

  // Sets the rates of the GP9 serial port and sensors
  void set_sensor_baud_rate(float baud);
  void set_sensor_baud_rate(float baud, float gps_baud, bool gps, bool sat);
  void set_raw_rate(uint8_t accel_raw_rate, uint8_t gyro_raw_rate, uint8_t mag_raw_rate);
  void set_all_raw_rate(uint8_t rate);
  void set_all_raw_rate(uint8_t temp_rate, uint8_t rate);
  void set_processed_rate(uint8_t accel_rate, uint8_t gyro_rate, uint8_t mag_rate);
  void set_all_processed_rate(uint8_t rate);
  void set_quaternion_rate(uint8_t rate);
  void set_euler_rate(uint8_t rate);
  void set_position_rate(uint8_t rate);
  void set_velocity_rate(uint8_t rate);
  void set_pose_rate(uint8_t rate);
  void set_health_rate(float baud);
  void set_gyro_bias_rate(uint8_t rate);

  // Sets the NMEA style rates
  void set_NMEA_health_rate(int8_t baud);
  void set_NMEA_pose_rate(int8_t baud);
  void set_NMEA_attitude_rate(int8_t baud);
  void set_NMEA_sensor_rate(int8_t baud);
  void set_NMEA_rates_rate(int8_t baud);
  void set_NMEA_GPS_pose_rate(int8_t baud);
  void set_NMEA_quaternion_rate(int8_t baud);

  void set_misc_ssettings(bool pps, bool zg, bool q, bool mag);
  void set_home_north(float north);
  void set_home_east(float east);
  void set_home_up(float up);
  void set_gyro_trim(float trim_x, float trim_y, float trim_z);
  void soft_iron_magnetometer_calibration(float (*array)[3][3]);
  void hard_iron_magnetometer_calibration(float bias_x, float bias_y, float bias_z);
  void accelerometer_misalignment_compensation(float (*array)[3][3]);
  void accelerometer_calibration(float bias_x, float bias_y, float bias_z);

  char* get_firmware_revision();
  void save_configs_to_flash();
  void zero_gyros();
  void set_home_position();
  void set_mag_reference();
  void calibrate_accelerometers();
  void reset_kalman_filter();
  void factory_reset();

  // HEALTH variables
  int16_t hdop;
  uint8_t sats_used, sats_in_view, ovf, gps_st, press, accel, gyro, mag, gps;

  // RAW Variables
  int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;
  int16_t accel_raw_x, accel_raw_y, accel_raw_z;
  int16_t mag_raw_x, mag_raw_y, mag_raw_z;
  float temp_time, temp1, temp2;
  float gyro_raw_time, accel_raw_time, mag_raw_time;
  float press_time, temp_raw1, temp_raw2, temp;
  uint32_t press_raw;

  // PROCESSED Variables
  float gyro_x, gyro_y, gyro_z, gyro_time;
  float accel_x, accel_y, accel_z, accel_time;
  float mag_x, mag_y, mag_z, mag_time;

  // TEMPERATURE and PRESSION
  float press_alt, press_proc_time, temp_proc1, temp_proc2, temp_proc_time;

  // EULER Variables
  int16_t roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
  float euler_time;

  // QUATERNION Variables
  int16_t quat_a, quat_b, quat_c, quat_d;
  float quat_time;

  // POSITION and VELOCITY Variables
  float north_pos, east_pos, up_pos, pos_time;
  float north_vel, east_vel, up_vel, vel_time;

  // GPS Variables
  float lattitude, longitude, altitude, course, speed, gps_time;
  float gps_date;
  uint8_t res, year, month,day;

  // SAT Variables
  float satellite_id0, satellite_SNR0, satellite_id1, satellite_SNR1, satellite_id2, satellite_SNR2, satellite_id3, satellite_SNR3, satellite_id4, satellite_SNR4, satellite_id5, satellite_SNR5, satellite_id6, satellite_SNR6, satellite_id7, satellite_SNR7, satellite_id8, satellite_SNR8, satellite_id9, satellite_SNR9, satellite_id10, satellite_SNR10, satellite_id11, satellite_SNR11;

  // BIAS Variables
  float gyro_bias_x, gyro_bias_y, gyro_bias_z;
  float bias_x, bias_y, bias_z;

  // QUAT VARIANCE variables
  float quat_a_var, quat_b_var, quat_c_var, quat_d_var;

    //////////////////////////////////
   //      INTERNAL FUNCTIONS      //
  //////////////////////////////////

private:

  bool checksum();
  void save();
  float read_register_as_float(int firstByte);
  
  // Placeholder for parsing binary packets
  int state;

  // Unscoped enumeration
  enum {STATE_ZERO,STATE_S,STATE_SN,STATE_SNP,STATE_PT,STATE_DATA,STATE_CHK1,STATE_CHK0};
  
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
  byte checksum1;                               // First byte of checksum
  byte checksum0;                               // Second byte of checksum
  uint16_t checksummer  = (checksum1<<8) | checksum0; // Combine the checksums
  unsigned short checksum10;          // Checksum received from packet
  unsigned short computed_checksum;             // Checksum computed from bytes received
};

#endif
