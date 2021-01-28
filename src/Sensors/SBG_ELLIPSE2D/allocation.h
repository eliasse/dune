#ifndef ALLOCATION_h
#define ALLOCATION

long lastMicros;

SBG_DATA SBG_Ellipse::AllocationData(uint8_t MSG, uint8_t CLASS,uint8_t DATA[])
{
//----------------------------------------------------------------------
// SBG_ECOM_CLASS_CMD_O
//----------------------------------------------------------------------
    if (CLASS == 0x10 && MSG == 0x00)  // ACKNOWLEDGED
    {
        ACK.cmdId     = DATA[0];
        ACK.classId   = DATA[1];
        ACK.errorCode = DATA[3] * 256 + DATA[2];

        // ouputSerial.print("ACK - ");
        // ouputSerial.print("CLASS: ");      ouputSerial.print(ACK.classId,HEX);
        // ouputSerial.print(" CMD: ");       ouputSerial.print(ACK.cmdId);
        // ouputSerial.print(" ErrorCode: "); ouputSerial.println(ACK.errorCode);
    }
//----------------------------------------------------------------------
    if (CLASS == 0x10 && MSG == 0x05)  // INITIAL PARAMETERS
    {
        InitPar.initLat     = *(double*)   ParseArray(0,8,DATA);
        InitPar.initLong    = *(double*)   ParseArray(8,8,DATA);
        InitPar.initAlt     = *(double*)   ParseArray(16,8,DATA);
        InitPar.Year        = *(uint16_t*) ParseArray(24,2,DATA);
        InitPar.Month       = *(uint8_t*)  ParseArray(26,1,DATA);
        InitPar.Day         = *(uint8_t*)  ParseArray(27,1,DATA);
    }
//----------------------------------------------------------------------
    if (CLASS == 0x10 && MSG == 0x07)  // MOTION PROFILE
    {
        MotionProfile.IDvalue = *(uint32_t*) ParseArray(0,4,DATA);
        MotionProfile.Revision = *(uint32_t*) ParseArray(4,4,DATA);
    }
//----------------------------------------------------------------------
    if (CLASS == 0x10 && MSG == 11)  // MAGNETOMETER MODEL ID
    {
        magModelId        = *(uint32_t*)  ParseArray(0,4,DATA);
        magModelRevision  = *(uint32_t*)  ParseArray(4,4,DATA);
    }
//----------------------------------------------------------------------
    if (CLASS == 0x10 && MSG == 12)  // MAGNETOMETER REJECT MODE
    {
        magRejectMode     = DATA[0];
    }
//----------------------------------------------------------------------
    if (CLASS == 0x10 && MSG == 15)  // COMPUTE MAGNETOMETER CALIB
    {

        calib.quality          = DATA[0];
        calib.confidence       = DATA[1];
        calib.advancedStatus   = *(uint16_t*)  ParseArray(2,2,DATA);
        calib.beforeMeanError  = *(float*)     ParseArray(4,4,DATA);
        calib.beforeStdError   = *(float*)     ParseArray(8,4,DATA);
        calib.beforeMaxError   = *(float*)     ParseArray(12,4,DATA);
        calib.afterMeanError   = *(float*)     ParseArray(16,4,DATA);
        calib.afterStdError    = *(float*)     ParseArray(20,4,DATA);
        calib.afterMaxError    = *(float*)     ParseArray(24,4,DATA);
        calib.meanAccuracy     = *(float*)     ParseArray(28,4,DATA);
        calib.stdAccuracy      = *(float*)     ParseArray(32,4,DATA);
        calib.maxAccuracy      = *(float*)     ParseArray(36,4,DATA);
        calib.numPoints        = *(uint16_t*)  ParseArray(40,2,DATA);
        calib.maxNumPoints     = *(uint16_t*)  ParseArray(42,2,DATA);

        for (int i=0;i<3;i++)
            MagCalibOffset[i]  = *(float*) ParseArray(44 + i*4,4,DATA);

        for (int i=0;i<9;i++)
            MagCalibMatrix[i]  = *(float*) ParseArray(56 + i*4,4,DATA);
    }
//----------------------------------------------------------------------
    if (CLASS == 0x10 && MSG == 23)  // UART PORT BAUDRATE
    {
        UART.portID         = *(uint8_t*)  ParseArray(0,1,DATA);
        UART.baudRate       = *(uint32_t*) ParseArray(1,4,DATA);
        UART.mode           = *(uint8_t*)  ParseArray(5,1,DATA);
    }
//----------------------------------------------------------------------
    if (CLASS == 0x10 && MSG == 30)  // OUTPUT CONFIGURATION
    {
        OutputConf.outputPortId   = *(uint8_t*)  ParseArray(0,1,DATA);
        OutputConf.msgId          = *(uint8_t*)  ParseArray(1,1,DATA);
        OutputConf.classId        = *(uint8_t*)  ParseArray(2,1,DATA);
        OutputConf.outputMode     = *(uint16_t*) ParseArray(3,2,DATA);
    }

//----------------------------------------------------------------------
// SBG_ECOM_CLASS_LOG_ECOM_O
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x01) // GENERAL STATUS
    {
        status.time_stamp        = *(uint32_t*) ParseArray(0,4,DATA);
        status.genStatus         = *(uint16_t*) ParseArray(4,2,DATA);
        status.comStatus         = *(uint32_t*) ParseArray(8,4,DATA);
        status.aidingStatus      = *(uint32_t*)  ParseArray(12,4,DATA);
        status.upTimeSec         = *(uint32_t*)  ParseArray(22,4,DATA);
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x02) // UTC TIME
    {
        utc.time_stamp           = *(uint32_t*) ParseArray(0,4,DATA);
        utc.clock_status         = *(uint16_t*) ParseArray(4,2,DATA);
        utc.year_                = *(uint16_t*) ParseArray(6,2,DATA);
        utc.month_               = *(uint8_t*)  ParseArray(8,1,DATA);
        utc.day_                 = *(uint8_t*)  ParseArray(9,1,DATA);
        utc.hour_                = *(uint8_t*)  ParseArray(10,1,DATA);
        utc.min_                 = *(uint8_t*)  ParseArray(11,1,DATA);
        utc.sec_                 = *(uint8_t*)  ParseArray(12,1,DATA);
        utc.nanosec              = *(uint32_t*) ParseArray(13,4,DATA);
        utc.gps_tow              = *(uint32_t*) ParseArray(17,4,DATA);
        return UTC_TIME_DATA;
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x03) // IMU DATA
    {
		    imu.time_stamp           = *(uint32_t*) ParseArray(0,4,DATA);
        imu.imu_status           = *(uint16_t*) ParseArray(4,2,DATA);
        imu.accel_x              = *(float*)    ParseArray(6,4,DATA);
        imu.accel_y              = *(float*)    ParseArray(10,4,DATA);
        imu.accel_z              = *(float*)    ParseArray(14,4,DATA);
        imu.gyro_x               = *(float*)    ParseArray(18,4,DATA);
        imu.gyro_y               = *(float*)    ParseArray(22,4,DATA);
        imu.gyro_z               = *(float*)    ParseArray(26,4,DATA);
        imu.temp                 = *(float*)    ParseArray(30,4,DATA);
        imu.delta_vel_x          = *(float*)    ParseArray(34,4,DATA);
        imu.delta_vel_y          = *(float*)    ParseArray(38,4,DATA);
        imu.delta_vel_z          = *(float*)    ParseArray(42,4,DATA);
        imu.delta_angle_x        = *(float*)    ParseArray(46,4,DATA);
        imu.delta_angle_y        = *(float*)    ParseArray(50,4,DATA);
        imu.delta_angle_z        = *(float*)    ParseArray(54,4,DATA);
        return IMU_DATA;
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x04) // MAGNETOMETER
    {
		    mag.time_stamp           = *(uint32_t*) ParseArray(0,4,DATA);
        mag.mag_status           = *(uint16_t*) ParseArray(4,2,DATA);
        mag.mx                   = *(float*)    ParseArray(6,4,DATA);
        mag.my                   = *(float*)    ParseArray(10,4,DATA);
        mag.mz                   = *(float*)    ParseArray(14,4,DATA);
        mag.accel_x              = *(float*)    ParseArray(18,4,DATA);
        mag.accel_y              = *(float*)    ParseArray(22,4,DATA);
        mag.accel_z              = *(float*)    ParseArray(26,4,DATA);
        return MAGNETOMETER_DATA;
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x06) // EKF EULER
    {
		    euler.time_stamp           = *(uint32_t*) ParseArray(0,4,DATA);
        euler.roll                 = *(float*)    ParseArray(4,4,DATA);
        euler.pitch                = *(float*)    ParseArray(8,4,DATA);
        euler.yaw                  = *(float*)    ParseArray(12,4,DATA);
        euler.roll_acc             = *(float*)    ParseArray(16,4,DATA);
        euler.pitch_acc            = *(float*)    ParseArray(20,4,DATA);
        euler.yaw_acc              = *(float*)    ParseArray(24,4,DATA);
        euler.solution_status      = *(uint32_t*) ParseArray(28,4,DATA);
        return EKF_EULER_DATA;
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x07) // EKF QUATERNION
    {
        quat.time_stamp           = *(uint32_t*) ParseArray(0,4,DATA);
        quat.q0                   = *(float*)    ParseArray(4,4,DATA);
        quat.q1                   = *(float*)    ParseArray(8,4,DATA);
        quat.q2                   = *(float*)    ParseArray(12,4,DATA);
        quat.q3                   = *(float*)    ParseArray(16,4,DATA);
        quat.roll_acc             = *(float*)    ParseArray(20,4,DATA);
        quat.pitch_acc            = *(float*)    ParseArray(24,4,DATA);
        quat.yaw_acc              = *(float*)    ParseArray(28,4,DATA);
        quat.solution_status      = *(uint32_t*) ParseArray(32,4,DATA);
        return EKF_QUAT_DATA;
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x08) // EKF NAVIGATION POSITION VELOCITY
    {
        nav.time_stamp               = *(uint32_t*) ParseArray(0,4,DATA);
        nav.vel_n                    = *(float*)    ParseArray(4,4,DATA);   // Velocity (m/s) in North direction
        nav.vel_e                    = *(float*)    ParseArray(8,4,DATA);   // Velocity (m/s) in East direction
        nav.vel_d                    = *(float*)    ParseArray(12,4,DATA);  // Velocity (m/s) in Down direction
        nav.vel_acc_n                = *(float*)    ParseArray(16,4,DATA);  // 1sigma accuracy velocity (m/s) in North direction
        nav.vel_acc_e                = *(float*)    ParseArray(20,4,DATA);  // 1sigma accuracy velocity (m/s) in East direction
        nav.vel_acc_d                = *(float*)    ParseArray(24,4,DATA);  // 1sigma accuracy velocity (m/s) in Down direction
        nav.lat                      = *(double*)   ParseArray(28,8,DATA);  // Latitude(deg)
        nav.lon                      = *(double*)   ParseArray(36,8,DATA);  // Longitude (deg)
        nav.alt                      = *(double*)   ParseArray(44,8,DATA);  // Altitude above MSL (m)
        nav.undulation               = *(float*)    ParseArray(52,4,DATA);  // Altitude difference between Geoid and ellipsoid (WGS-84 - MSL alt)
        nav.lat_acc                  = *(float*)    ParseArray(56,4,DATA);  // 1sigma latitude accuracy (m)
        nav.lon_acc                  = *(float*)    ParseArray(60,4,DATA);  // 1sigma lon accuracy (m)
        nav.alt_acc                  = *(float*)    ParseArray(64,4,DATA);  // 1sigma altitude accuracy (m)
        nav.solution_status          = *(uint32_t*) ParseArray(32,4,DATA);
        return EKF_NAV_POS_VEL_DATA;
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x09) // SHIP MOTION
    {
		    ShipMotion.time_stamp           = *(uint32_t*) ParseArray(0,4,DATA);
        ShipMotion.heave_period         = *(uint16_t*) ParseArray(4,4,DATA);
        ShipMotion.surge                = *(float*)    ParseArray(8,8,DATA);
        ShipMotion.sway                 = *(float*)    ParseArray(12,4,DATA);
        ShipMotion.heave                = *(float*)    ParseArray(16,4,DATA);
        ShipMotion.accel_x              = *(float*)    ParseArray(20,4,DATA);
        ShipMotion.accel_y              = *(float*)    ParseArray(24,4,DATA);
        ShipMotion.accel_z              = *(float*)    ParseArray(28,4,DATA);
        ShipMotion.vel_x                = *(float*)    ParseArray(32,4,DATA);
        ShipMotion.vel_y                = *(float*)    ParseArray(36,4,DATA);
        ShipMotion.vel_z                = *(float*)    ParseArray(40,4,DATA);
        ShipMotion.heave_status         = *(float*)    ParseArray(44,2,DATA);
        return SHIP_MOTION_DATA;
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x0D) // GPS Velocity
    {
		    GPSVel.time_stamp               = *(uint32_t*) ParseArray(0,4,DATA);
        GPSVel.gps_vel_status           = *(uint32_t*) ParseArray(4,4,DATA);
        GPSVel.gps_tow                  = *(uint32_t*) ParseArray(8,4,DATA);
        GPSVel.vel_n                    = *(float*)    ParseArray(12,4,DATA); // Velocity (m/s) in North direction
        GPSVel.vel_e                    = *(float*)    ParseArray(16,4,DATA); // Velocity (m/s) in East direction
        GPSVel.vel_d                    = *(float*)    ParseArray(20,4,DATA); // Velocity (m/s) in Down direction
        GPSVel.vel_acc_n                = *(float*)    ParseArray(24,4,DATA); // 1sigma accuracy velocity (m/s) in North direction
        GPSVel.vel_acc_e                = *(float*)    ParseArray(28,4,DATA); // 1sigma accuracy velocity (m/s) in East direction
        GPSVel.vel_acc_d                = *(float*)    ParseArray(32,4,DATA); // 1sigma accuracy velocity (m/s) in Down direction
        GPSVel.cog                      = *(float*)    ParseArray(36,4,DATA); // True course over ground (deg), 0 to 360 deg
        GPSVel.cog_acc                  = *(float*)    ParseArray(40,4,DATA); // 1sigma course accuracy (deg)
        return GPS_VEL_DATA;
    }
//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x0E) // GPS Position
    {
		    GPSPos.time_stamp               = *(uint32_t*) ParseArray(0,4,DATA);
        GPSPos.gps_pos_status           = *(uint32_t*) ParseArray(4,4,DATA);
        GPSPos.gps_tow                  = *(uint32_t*) ParseArray(8,4,DATA);
        GPSPos.lat                      = *(double*)   ParseArray(12,8,DATA); // Latitude(deg)
        GPSPos.lon                      = *(double*)   ParseArray(20,8,DATA); // Longitude (deg)
        GPSPos.alt                      = *(double*)   ParseArray(28,8,DATA); // Altitude above MSL (m)
        GPSPos.undulation               = *(float*)    ParseArray(36,4,DATA); // Altitude difference between Geoid and ellipsoid (WGS-84 - MSL alt)
        GPSPos.lat_acc                  = *(float*)    ParseArray(40,4,DATA); // 1sigma latitude accuracy (m)
        GPSPos.lon_acc                  = *(float*)    ParseArray(44,4,DATA); // 1sigma lon accuracy (m)
        GPSPos.alt_acc                  = *(float*)    ParseArray(48,4,DATA); // 1sigma altitude accuracy (m)
        GPSPos.num_sat                  = *(uint8_t*)  ParseArray(52,1,DATA); // Number of satelites used for solution
        GPSPos.base_station_id          = *(uint16_t*) ParseArray(54,2,DATA); // ID of base used for DGPS/RTK
        GPSPos.diff_age                 = *(uint16_t*) ParseArray(56,2,DATA); // Differential data age
        return GPS_POS_DATA;
    }

//----------------------------------------------------------------------
    if (CLASS == 0x00 && MSG == 0x0F) // GPS TRUE HEADING
    {
		    GPSHdt.time_stamp               = *(uint32_t*) ParseArray(0,4,DATA);
        GPSHdt.gps_hdt_status           = *(uint16_t*) ParseArray(4,2,DATA);
        GPSHdt.gps_tow                  = *(uint32_t*) ParseArray(6,4,DATA);
        GPSHdt.true_heading             = *(float*)    ParseArray(10,4,DATA); // True heading (deg)
        GPSHdt.true_heading_acc         = *(float*)    ParseArray(14,4,DATA); // True heading accuracy (deg)
        GPSHdt.gps_pitch                = *(float*)    ParseArray(18,4,DATA); // Pitch angle between antennas (deg)
        GPSHdt.gps_pitch_acc            = *(float*)    ParseArray(22,4,DATA); // Estimated GPS antenna pitch accuracy (deg)
        return GPS_TRUE_HEADING_DATA;
    }
    return NOT_IMPLEMENTED;
}
//----------------------------------------------------------------------
#endif
