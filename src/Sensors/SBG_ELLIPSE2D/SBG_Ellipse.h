/*
  SBG_Ellipse AHRS library
  Created by Lazaro Moratelli Jr, Dec 2018.
*/

#ifndef SBG_Ellipse_H_
#define SBG_Ellipse_H_

// Inclusions
#include "stdint.h"
#include "Stream.h"

  class SBG_Ellipse
  {
  //----------------------------------------------------------------------
  // PUBLIC AND PRIVATE VARIABLE/PARAMETERS
  //----------------------------------------------------------------------
  public:
    struct SBG_ECOM_LOG_STATUS
    {
      uint32_t time_stamp, comStatus, aidingStatus, upTimeSec;
      uint16_t genStatus;
    } status;

    struct SBG_ECOM_LOG_UTC_TIME
    {
      uint32_t time_stamp, nanosec, gps_tow;
      uint16_t clock_status, year_;
      uint8_t month_, day_, hour_, min_, sec_;
    } utc;

    struct SBG_ECOM_LOG_IMU_DATA
    {
      uint32_t time_stamp;
      uint16_t imu_status;
      float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
      float temp, delta_vel_x, delta_vel_y,delta_vel_z;
      float delta_angle_x, delta_angle_y, delta_angle_z;
    } imu;

    struct SBG_ECOM_LOG_MAG
    {
      uint32_t time_stamp;
      uint16_t mag_status;
      float mx, my, mz, accel_x, accel_y, accel_z;
    } mag;

    struct SBG_ECOM_EKF_EULER
    {
      uint32_t time_stamp, solution_status;
      float roll, pitch, yaw, roll_acc, pitch_acc, yaw_acc;
    } euler;

    struct SBG_ECOM_EKF_QUAT
    {
      uint32_t time_stamp, solution_status;
      float q0, q1, q2, q3, roll_acc, pitch_acc, yaw_acc;
    } quat;

    struct SBG_ECOM_EKF_NAV
    {
      uint32_t time_stamp, solution_status;
      double lat, lon, alt;
      float vel_n, vel_e, vel_d, vel_acc_n, vel_acc_e, vel_acc_d, undulation, lat_acc, lon_acc, alt_acc;
    } nav;

    struct SBG_ECOM_CMD_MOTION_PROFILE_ID
    {
      uint8_t CLASS = 0x10, MSG = 0x07;
      uint32_t IDvalue, Revision;
    } MotionProfile;

    struct SBG_ECOM_CMD_ACK
    {
      uint8_t cmdId, classId, errorCode;
    } ACK;

    struct SBG_ECOM_CMD_INIT_PARAMETERS
    {
      double initLat, initLong, initAlt;
      uint16_t Year;
      uint8_t Month, Day;
    } InitPar;

    struct SBG_ECOM_CMD_UART_CONF
    {
      uint32_t baudRate;
      uint8_t portID, mode;
    } UART;

    struct OUTPUT_CONF
    {
      int outputPortId, msgId, classId;
      uint16_t outputMode;
    } OutputConf;

    struct SHIP_MOTION
    {
      uint32_t time_stamp;
      uint16_t heave_status;
      float heave_period, surge, sway, heave;
      float accel_x, accel_y, accel_z, vel_x, vel_y, vel_z;
    } ShipMotion;

    struct SBG_ECOM_LOG_GPS1_VEL
    {
      uint32_t time_stamp;
      uint32_t gps_vel_status;
      uint32_t gps_tow;
      float vel_n, vel_e, vel_d;
      float vel_acc_n, vel_acc_e, vel_acc_d;
      float cog, cog_acc;
    } GPSVel;

    struct SBG_ECOM_LOG_GPS1_POS
    {
      uint32_t time_stamp;
      uint32_t gps_pos_status;
      uint32_t gps_tow;
      double lat, lon, alt;
      float undulation; //altitude difference between the geoid and the elipsoid (WGS84 alt - MSL)
      float lat_acc, lon_acc, alt_acc;
      uint8_t num_sat; //Number of staelites used in GNSS solution
      uint16_t base_station_id;// ID of DGPS/rtk base station in use
      uint16_t diff_age;
    } GPSPos;

    struct SGB_ECOM_GPS1_HDT
    {
      uint32_t time_stamp, gps_tow;
      uint16_t gps_hdt_status;
      float true_heading, true_heading_acc, gps_pitch, gps_pitch_acc;
    } GPSHdt;

    uint32_t magModelId, magModelRevision;           // SBG_ECOM_CMD_MAGNETOMETER_MODEL_ID
    uint8_t magRejectMode;                           // SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE
    float MagCalibOffset[3], MagCalibMatrix[9];      // SBG_ECOM_CMD_SET_MAG_CALIB
    float magFieldStrength = 0.0;

    struct CALIBRATION
    {
      uint8_t quality, confidence;
      uint16_t advancedStatus;
      float beforeMeanError, beforeStdError, beforeMaxError;
      float afterMeanError, afterStdError, afterMaxError;
      float meanAccuracy, stdAccuracy, maxAccuracy;
      uint16_t numPoints, maxNumPoints;
    } calib;

  private:
    Stream* port;
    const double pi = 3.14159265358979323846;
    uint8_t BufferData[256];                                  //SerialPort Data
    uint8_t DATA[256], FRAME[256];                                        //Payload
    uint8_t LENGTH, CLASS, MSG, LEN[2], CRC[2];           //Fields
    uint8_t SYNC1 = 0xFF, SYNC2 = 0x5A, ETX = 0x33, ETXchk;   //header and end
    int counter = 0, ctr = 0;
    uint16_t CRCInt, crc;

    uint8_t Output2[4];
    uint8_t OutputArray[140];
    uint8_t Frame[256];



  //----------------------------------------------------------------------
  // PUBLIC AND PRIVATE MEMBERS
  //----------------------------------------------------------------------
  public:
  	SBG_Ellipse(); // Empty constructor
  	void begin(Stream *SerialPort);
    void ReadData();

    void GET_MOTION_PROFILE_ID();
    void SET_MOTION_PROFILE_ID(uint8_t ID);
    void SETTINGS_ACTION(uint8_t ID);
    void GET_INIT_PARAMETERS();
    void SET_INIT_PARAMETERS();
    void GET_UART_CONF(int portID);
    void SET_UART_CONF(int portID,uint32_t baudRate,int mode);
    void GET_OUTPUT_CONF(int outputPortId, int msgId, int classId);
    void SET_OUTPUT_CONF(int outputPortId, int msgId, int classId,uint16_t outputMode);
    void GET_MAG_MODEL_ID();
    void SET_MAG_MODEL_ID(uint32_t magModelId);
    void GET_MAG_REJEC_MODE();
    void SET_MAG_REJEC_MODE(uint8_t magRejectMode);
    void SET_MAG_CALIB();
    void START_MAG_CALIB(uint8_t mode,uint8_t bandwidth);
    void COMPUTE_MAG_CALIB();
    void GET_GNSS1_LEVER();
    void SET_GNSS1_LEVER(float leverArmX, float leverArmY, float leverArmZ, float pitchOffset, float yawOffset, float antennaDistance);
    void GET_GNSS1_REJEC_MODE();
    void SET_GNSS1_REJEC_MODE(uint8_t posRejectMode, uint8_t velRejectMode, uint8_t hdtRejectMode);
    void calculateFieldStrength();
    float ToDeg(float x);
    float unwrap2Pi(float x);

  private:
    uint16_t CRCcheck();
    void AllocationData(uint8_t MSG, uint8_t CLASS, uint8_t DATA[]);
    uint8_t *ParseArray(int offset,int length,uint8_t Array[]);
    float ByteToFloat(int a, int b, uint8_t DATA[]);
    void WriteFrame();

  };

#endif /* SBG_Ellipse_H_ */
