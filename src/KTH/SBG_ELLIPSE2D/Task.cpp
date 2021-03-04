//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "SBG_Ellipse.h"


namespace KTH
{

  namespace SBG_ELLIPSE2D
  {
    using DUNE_NAMESPACES;

    //Static stuff

    struct Arguments
    {
      // Serial port device.
      std::string uart_dev;
      // Serial port baud rate.
      unsigned uart_baud;
    };

    unsigned int id_true_heading;

    struct Task: public Tasks::Task
    {
      //Local variables
      Arguments m_args;
      SBG_Ellipse ahrs;
      Stream* serialPort;

      //variables to keep track of when new data is received
      double last_lat, last_lon;
      float last_true_heading, last_gps_pitch;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx)
      {
        param("Serial Port - Device", m_args.uart_dev)
                            .defaultValue("/dev/ttyUSB-1")
                            .description("Serial port device (used to communicate with the actuator)");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("9600")
        .description("Serial port baud rate");


      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
        resolveEntity(getEntityId());
        id_true_heading = reserveEntity("SBG ELLIPSE TRUE HEADING");
      }

      void
      onResourceAcquisition(void)
      {

      }

      void
      onResourceRelease(void)
      {
      }

      void
      onResourceInitialization(void)
      {
          //Start sbg etc
          std::cout << "SBG_Ellipse: Serial port: " << m_args.uart_dev << ", baud:" << m_args.uart_baud << std::endl;
          serialPort = new Stream(m_args.uart_dev, m_args.uart_baud);
          ahrs.begin(serialPort);
      }

      void
      onMain(void)
      {
        while (!stopping())
        {
          //waitForMessages(0.0001);
          SBG_DATA sbg_data_type = ahrs.ReadData();
          switch (sbg_data_type) {
            case UTC_TIME_DATA:
            {

            }
            break;
            case IMU_DATA:
            {
              IMC::AngularVelocity msg_gyro;
              msg_gyro.x = ahrs.imu.gyro_x;
              msg_gyro.y = ahrs.imu.gyro_y;
              msg_gyro.z = ahrs.imu.gyro_z;
              msg_gyro.setSourceEntity(getEntityId());
              dispatch(msg_gyro);

              IMC::Acceleration msg_acc;
              msg_acc.x = ahrs.imu.accel_x;
              msg_acc.y = ahrs.imu.accel_y;
              msg_acc.z = ahrs.imu.accel_z;
              msg_acc.setSourceEntity(getEntityId());
              dispatch(msg_acc);
            }
            break;
            case MAGNETOMETER_DATA:
            {
              IMC::MagneticField msg;
              msg.x = ahrs.mag.mx;
              msg.y = ahrs.mag.my;
              msg.z = ahrs.mag.mz;
              msg.setSourceEntity(getEntityId());
              dispatch(msg);
            }
            break;
            case EKF_EULER_DATA:
            {
              IMC::EulerAngles msg;
              msg.phi = ahrs.euler.roll;
              msg.theta = ahrs.euler.pitch;
              msg.psi = ahrs.euler.yaw;
              msg.setSourceEntity(getEntityId());
              dispatch(msg);
            }
            break;
            case EKF_QUAT_DATA:
            {

            }
            break;
            case EKF_NAV_POS_VEL_DATA:
            {

            }
            break;
            case SHIP_MOTION_DATA:
            {

            }
            break;
            case GPS_VEL_DATA:
            {

            }
            break;
            case GPS_POS_DATA:
            {
              if(ahrs.GPSPos.lat != last_lat || ahrs.GPSPos.lon != last_lon) {
                IMC::GpsFix msg;
                msg.lat = ahrs.GPSPos.lat;
                msg.lon = ahrs.GPSPos.lon;
                msg.satellites = ahrs.GPSPos.num_sat;
                //TODO add the rest
                msg.setSourceEntity(getEntityId());
                dispatch(msg);
                ahrs.GPSPos.lat = last_lat;
                ahrs.GPSPos.lon = last_lon;
                std::cout << "SBG: New gps position" << std::endl;
              }
            }
            break;
            case GPS_TRUE_HEADING_DATA:
            {
              if(ahrs.GPSHdt.gps_pitch != last_gps_pitch || ahrs.GPSHdt.true_heading != last_true_heading) {
                IMC::EulerAngles msg;
                msg.theta = ahrs.GPSHdt.gps_pitch;
                msg.psi = ahrs.GPSHdt.true_heading;
                msg.setSourceEntity(id_true_heading);
                dispatch(msg);
                ahrs.GPSHdt.gps_pitch = last_gps_pitch;
                ahrs.GPSHdt.true_heading = last_true_heading;
                std::cout << "SBG: New true heading" << std::endl;
              }
            }
            break;
          }
        }
      }
    };
  }
}

DUNE_TASK
