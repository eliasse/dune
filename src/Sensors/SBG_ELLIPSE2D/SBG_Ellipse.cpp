// Inclusions
#include "SBG_Ellipse.h"
#include "Stream.h"
#include "allocation.h"
#include <cmath>
#include <iostream>

//----------------------------------------------------------------------
// CONSTRUCTOR AND SERIAL PORT
//----------------------------------------------------------------------
SBG_Ellipse::SBG_Ellipse(){
}
//----------------------------------------------------------------------
void SBG_Ellipse::begin(Stream *SerialPort)
{
  port = SerialPort;
}
//----------------------------------------------------------------------
void SBG_Ellipse::ReadData()
{

    if (port -> available())
    {
        if (counter >=255){                            // max boundary
            counter = 0;
            ctr     = 0;
        }

        BufferData[counter] = port -> read();
        std::cout << "SBG sent: " << (int)BufferData[counter]  << std::endl;

        if (ctr == 1)
        {
            if (BufferData[counter-2] == ETX && BufferData[counter-1] == SYNC1  && BufferData[counter] == SYNC2)
            {

                MSG    = BufferData[2];
                CLASS  = BufferData[3];

                LEN[0] = BufferData[4];
                LEN[1] = BufferData[5];
                LENGTH = LEN[1] * 256 + LEN[0];

                if (LENGTH>0){                           // Get payload
                    for (int i=0;i<LENGTH;i++){
                        DATA[i] = BufferData[6+i];
                    }
                }

                CRC[0] = BufferData[counter-4];
                CRC[1] = BufferData[counter-3];
                CRCInt = CRC[1] * 256 + CRC[0];
                crc = CRCcheck();

                if (crc == CRCInt){                     // check if message is ok
                    AllocationData(MSG,CLASS,DATA);
                    std::cout << "SBG data received" << std::endl;
                }

                ctr = 0;
                counter = 2;
                BufferData[counter-2] = ETX;
                BufferData[counter-1] = SYNC1;
                BufferData[counter]   = SYNC2;
            }
        }

        if (ctr == 0 && counter >=2)                    // looking header
        {
            if (BufferData[counter-2] == ETX && BufferData[counter-1] == SYNC1  && BufferData[counter] == SYNC2)
            {
                BufferData[0] = BufferData[counter-1];
                BufferData[1] = BufferData[counter];
                counter = 1;
                ctr = 1;                               // header found!
            }
        }

        counter = counter + 1;
    }
}
//----------------------------------------------------------------------
uint16_t SBG_Ellipse::CRCcheck()
{
  //MSG,CLASS,LEN,DATA,LENGTH
  uint8_t pBytesArray[256];
  uint16_t poly = 0x8408;
  uint8_t carry;
  uint8_t i_bits;
  uint16_t j;
  int BufferSize;

  crc = 0;

  BufferSize = 4 + LENGTH;
  pBytesArray[0] = MSG;
  pBytesArray[1] = CLASS;
  pBytesArray[2] = LEN[0];
  pBytesArray[3] = LEN[1];

  if (LENGTH >0)
  {
      for (int i=4; i<BufferSize;i++){
        pBytesArray[i] = DATA[i-4];
      }
  }

  for (j =0; j < BufferSize; j++)
  {
    crc = crc ^ pBytesArray[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
        carry = crc & 1;
        crc = crc / 2;
        if (carry)
        {
            crc = crc^poly;
        }
    }
  }

  return crc;
}
//----------------------------------------------------------------------
uint8_t *SBG_Ellipse::ParseArray(int offset,int length,uint8_t Array[])
{
    for (int i = 0; i<=offset+length-1; i++)
        OutputArray[i] = Array[offset+i];
    return OutputArray;
}
//----------------------------------------------------------------------
void SBG_Ellipse::WriteFrame()
{
    Frame[0]   = SYNC1;
    Frame[1]   = SYNC2;
    Frame[2]   = MSG;
    Frame[3]   = CLASS;

    LEN[0]     = (LENGTH & 0xFF);
    LEN[1]     = (LENGTH >> 8) & 0xFF;

    Frame[4]   = LEN[0];
    Frame[5]   = LEN[1];

    int offset = 6;

    if (LENGTH > 0)
    {
        for (int i=0;i<LENGTH;i++)
        {
            Frame[offset] = DATA[i];
            offset = offset + 1;
        }
    }

    crc     = CRCcheck();
    CRC[0]  = (crc & 0xFF);
    CRC[1]  = (crc >> 8) & 0xFF;

    Frame[offset]    = CRC[0];
    Frame[offset+1]  = CRC[1];
    Frame[offset+2]  = ETX;

    port -> write(Frame,offset+3);
}

//----------------------------------------------------------------------
void SBG_Ellipse::GET_MOTION_PROFILE_ID()
{
    CLASS = 0x10; MSG = 0x07; LENGTH = 0;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SET_MOTION_PROFILE_ID(uint8_t ID)
{
    CLASS = 0x10; MSG = 0x07; LENGTH = 4;

    DATA[0] = ID;
    DATA[1] = 0;
    DATA[2] = 0;
    DATA[3] = 0;

    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SETTINGS_ACTION(uint8_t ID)
{
    //REBOOT ONLY              - 0
    //SAVE_SETTINGS            - 1
    //RESTORE_DEFAULT_SETTINGS - 2
    CLASS = 0x10; MSG = 0x01; LENGTH = 1;
    DATA[0] = ID;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::GET_INIT_PARAMETERS()
{
    CLASS = 0x10; MSG = 0x05; LENGTH = 0;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SET_INIT_PARAMETERS()
{
    union{
        double value;  uint8_t Bytes[8];
    } Lat, Long, Alt;
    union{
        uint16_t value; uint8_t Bytes[2];
    } Year;

    CLASS = 0x10; MSG = 0x05; LENGTH = 28;

    Lat.value  = InitPar.initLat;
    Long.value = InitPar.initLong;
    Alt.value  = InitPar.initAlt;
    Year.value = InitPar.Year;

    for (int i = 0;i<8;i++)
    {
        DATA[i]    = Lat.Bytes[i];
        DATA[i+8]  = Long.Bytes[i];
        DATA[i+16] = Alt.Bytes[i];
    }

    DATA[24] = Year.Bytes[0];
    DATA[25] = Year.Bytes[1];
    DATA[26] = InitPar.Month;
    DATA[27] = InitPar.Day;

    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::GET_UART_CONF(int portID)
{
    /*  PortID numbers
        SBG_ECOM_PORT_A  -  0  -  Main communication interface. Full duplex
        SBG_ECOM_PORT_B  -  1  -  Auxiliary input interface for RTCM
        SBG_ECOM_PORT_C  -  2  -  Auxiliary communication interface. Full duplex
        SBG_ECOM_PORT_D  -  3  -  Auxiliary input interface
        SBG_ECOM_PORT_E  -  4  -  Auxiliary inout interface*/
    CLASS = 0x10; MSG = 23; LENGTH = 1; DATA[0] = portID;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SET_UART_CONF(int portID,uint32_t baudRate,int mode)
{
    /* UART mode
       SBG_ECOM_UART_MODE_OFF  -  0  -  This interface is turned OFF.
       SBG_ECOM_UART_MODE_232  -  1  -  This interface is using RS-232 communications
       SBG_ECOM_UART_MODE_422  -  2  -  This interface is using RS-422 communications*/
    union{
        uint32_t value; uint8_t Bytes[4];
    } BR;
    BR.value = baudRate;

    CLASS = 0x10; MSG = 23; LENGTH = 6;
    DATA[0] = portID;
    DATA[1] = BR.Bytes[0];
    DATA[2] = BR.Bytes[1];
    DATA[3] = BR.Bytes[2];
    DATA[4] = BR.Bytes[3];
    DATA[5] = mode;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::GET_OUTPUT_CONF(int outputPortId, int msgId, int classId)
{
    /*  SBG_ECOM_OUTPUT_PORT_A - 0 - Main output port
        SBG_ECOM_OUTPUT_PORT_C - 2 - Auxiliary output interface for ELLIPSE-E
        SBG_ECOM_OUTPUT_PORT_E - 4 - Miscellaneous output interface for B1 models */

    CLASS = 0x10; MSG = 30; LENGTH = 3;
    DATA[0] = outputPortId;
    DATA[1] = msgId;
    DATA[2] = classId;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SET_OUTPUT_CONF(int outputPortId, int msgId, int classId,uint16_t outputMode)
{
    /*  SBG_ECOM_OUTPUT_MODE_DISABLED  - 0   - Output is disabled
        SBG_ECOM_OUTPUT_MODE_MAIN_LOOP - 1   - Output is generated at 200Hz
        SBG_ECOM_OUTPUT_MODE_DIV_2     - 2   - Output is generated at 100Hz
        SBG_ECOM_OUTPUT_MODE_DIV_4     - 4   - Output is generated at 50Hz
        SBG_ECOM_OUTPUT_MODE_DIV_8     - 8   - Output is generated at 25Hz
        SBG_ECOM_OUTPUT_MODE_DIV_10    - 10  - Output is generated at 20Hz
        SBG_ECOM_OUTPUT_MODE_DIV_20    - 20  - Output is generated at 10Hz
        SBG_ECOM_OUTPUT_MODE_DIV_40    - 40  - Output is generated at 5Hz
        SBG_ECOM_OUTPUT_MODE_DIV_200   - 200 - Output is generated at 1Hz*/

    CLASS = 0x10; MSG = 30; LENGTH = 5;
    DATA[0]  = outputPortId;
    DATA[1]  = msgId;
    DATA[2]  = classId;
    DATA[3]  = (outputMode & 0xFF);
    DATA[4]  = (outputMode >> 8) & 0xFF;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::GET_MAG_MODEL_ID()
{
    /*  SBG_ECOM_MAG_MODEL_NORMAL 201 Should be used in most applications
        SBG_ECOM_MAG_MODEL_NOISY_MAG_TOLERANT 202 Should be used in disturbed magnetic environment*/
    CLASS = 0x10; MSG = 11; LENGTH = 0;

    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SET_MAG_MODEL_ID(uint32_t magModelId)
{
    /*  SBG_ECOM_MAG_MODEL_NORMAL             - 201 - Should be used in most applications
        SBG_ECOM_MAG_MODEL_NOISY_MAG_TOLERANT - 202 - Should be used in disturbed magnetic environment*/
    union{
        uint32_t value; uint8_t Bytes[4];
    } Data;
    Data.value = magModelId;

    CLASS = 0x10; MSG = 11; LENGTH = 4;
    for (int i=0;i<4;i++)
    {
        DATA[i]   = Data.Bytes[i];
    }
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::GET_MAG_REJEC_MODE()
{
    CLASS = 0x10; MSG = 12; LENGTH = 0;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SET_MAG_REJEC_MODE(uint8_t magRejectMode)
{
    /*  SBG_ECOM_NEVER_ACCEPT_MODE  - 0 - Measurement is not taken into account
        SBG_ECOM_AUTOMATIC_MODE     - 1 - Measurement is rejected if inconsistent with current estimate
        SBG_ECOM_ALWAYS_ACCEPT_MODE - 2 - Measurement is always accepted */

    CLASS = 0x10; MSG = 12; LENGTH = 1; DATA[0] = magRejectMode;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SET_MAG_CALIB() //SBG_ECOM_CMD_SET_MAG_CALIB
{
    union{
        float Element;
        uint8_t Bytes[4];
    } value;

    int k = 0;
    for (int i=0;i<3;i++){
        value.Element = MagCalibOffset[i];
        DATA[k] = value.Bytes[0];
        DATA[k+1] = value.Bytes[1];
        DATA[k+2] = value.Bytes[2];
        DATA[k+3] = value.Bytes[3];
        k=k+4;
    }
    k = 12;
    for (int i=0;i<9;i++){
        value.Element = MagCalibMatrix[i];
        DATA[k] = value.Bytes[0];
        DATA[k+1] = value.Bytes[1];
        DATA[k+2] = value.Bytes[2];
        DATA[k+3] = value.Bytes[3];
        k=k+4;
    }

    CLASS = 0x10; MSG = 13; LENGTH = 48;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::START_MAG_CALIB(uint8_t mode,uint8_t bandwidth)
{
    /* MagCalibMode
        SBG_ECOM_MAG_CALIB_2D - 1
        SBG_ECOM_MAG_CALIB_3D - 2 */
    /* MagCalibBandwidth
        SBG_ECOM_MAG_CALIB_LOW_BW    - 0
        SBG_ECOM_MAG_CALIB_MEDIUM_BW - 1
        SBG_ECOM_MAG_CALIB_HIGH_BW   - 2 */

    CLASS = 0x10; MSG = 14; LENGTH = 2;
    DATA[0] = mode;
    DATA[1] = bandwidth;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::COMPUTE_MAG_CALIB()
{
    CLASS = 0x10; MSG = 15; LENGTH = 0;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::GET_GNSS1_LEVER()
{
    CLASS = 0x10; MSG = 18; LENGTH = 0;
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::SET_GNSS1_LEVER(float leverArmX, float leverArmY, float leverArmZ, float pitchOffset, float yawOffset, float antennaDistance)
{
    /*leverArmX - Distance in m in IMU X axis between IMU and Antenna
    leverArmY - Distance in m in IMU Y axis between IMU and Antenna
    leverArmZ - Distance in m in IMU Z axis between IMU and Antenna
    pitchOffset - Pitch offset in radian between the 2 antennas and IMU for dual antenna for dual antenna system. Leave to 0 for single amntenna
    yawOffset - Yaw offset in radian between the 2 antennas and IMU for dual antenna for dual antenna system. Leave to 0 for single amntenna
    */
    CLASS = 0x10; MSG = 18; LENGTH = 24;
    union{
        float value; uint8_t Bytes[4];
    } LAX;
    LAX.value = leverArmX;
    union{
        float value; uint8_t Bytes[4];
    } LAY;
    LAY.value = leverArmY;
    union{
        float value; uint8_t Bytes[4];
    } LAZ;
    LAZ.value = leverArmZ;
    union{
        float value; uint8_t Bytes[4];
    } PO;
    PO.value = pitchOffset;
    union{
        float value; uint8_t Bytes[4];
    } YO;
    YO.value = yawOffset;
    union{
        float value; uint8_t Bytes[4];
    } AD;
    AD.value = antennaDistance;

    DATA[0] = LAX.Bytes[0];
    DATA[1] = LAX.Bytes[1];
    DATA[2] = LAX.Bytes[2];
    DATA[3] = LAX.Bytes[3];
    DATA[4] = LAY.Bytes[0];
    DATA[5] = LAY.Bytes[1];
    DATA[6] = LAY.Bytes[2];
    DATA[7] = LAY.Bytes[3];
    DATA[8] = LAZ.Bytes[0];
    DATA[9] = LAZ.Bytes[1];
    DATA[10] = LAZ.Bytes[2];
    DATA[11] = LAZ.Bytes[3];
    DATA[12] = PO.Bytes[0];
    DATA[13] = PO.Bytes[1];
    DATA[14] = PO.Bytes[2];
    DATA[15] = PO.Bytes[3];
    DATA[16] = YO.Bytes[0];
    DATA[17] = YO.Bytes[1];
    DATA[18] = YO.Bytes[2];
    DATA[19] = YO.Bytes[3];
    DATA[20] = AD.Bytes[0];
    DATA[21] = AD.Bytes[1];
    DATA[22] = AD.Bytes[2];
    DATA[23] = AD.Bytes[3];
    WriteFrame();
}//----------------------------------------------------------------------
void SBG_Ellipse::GET_GNSS1_REJEC_MODE()
{
    CLASS = 0x10; MSG = 19; LENGTH = 0;
    WriteFrame();
}//----------------------------------------------------------------------
void SBG_Ellipse::SET_GNSS1_REJEC_MODE(uint8_t posRejectMode, uint8_t velRejectMode, uint8_t hdtRejectMode)
{
    /*  SBG_ECOM_NEVER_ACCEPT_MODE  - 0 - Measurement is not taken into account
    SBG_ECOM_AUTOMATIC_MODE     - 1 - Measurement is rejected if inconsistent with current estimate
    SBG_ECOM_ALWAYS_ACCEPT_MODE - 2 - Measurement is always accepted */

    CLASS = 0x10; MSG = 19; LENGTH = 4;
    DATA[0] = posRejectMode; //Postion rejection mode
    DATA[1] = velRejectMode; //Velocity rejection mode
    DATA[2] = 0;
    DATA[3] = hdtRejectMode; //True heading rejection mode
    WriteFrame();
}
//----------------------------------------------------------------------
void SBG_Ellipse::calculateFieldStrength() {
  magFieldStrength = pow((pow(mag.mx,2)+pow(mag.my,2)+pow(mag.mz,2)),0.5);
}
//----------------------------------------------------------------------
float SBG_Ellipse::ToDeg(float x) { return x*180/pi;}
//----------------------------------------------------------------------
float SBG_Ellipse::unwrap2Pi(float x) {
    while (x < 0.0) { x = x + 2*pi; }
    while (x > 2.0*pi) { x = x - 2.0*pi; }
    return x;
}
